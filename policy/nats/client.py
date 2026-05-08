"""NATS request/reply policy client.

Uses NATS for app-to-app policy inference on the Nova platform.
The policy service subscribes to a subject and replies with actions.

Wire protocol:
- **Scalars** (joints, IOs): sent as msgpack via request/reply on ``subject``
- **Images** (numpy arrays): published as PNG on ``subject.images.<name>``

The policy service keeps the latest image per camera and merges them
into the observation when a prediction request arrives.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Any

from policy.nats.wire import pack, pack_image
from policy.policy_client import PolicyClient
from policy.types import ActionChunk, PolicyResponse

if TYPE_CHECKING:
    import nats.aio.client
    from numpy.typing import NDArray

    from nova.types import RobotState
    from policy.schema import PolicySchema

logger = logging.getLogger(__name__)

_DEFAULT_SUBJECT = "nova.v2.cells.cell.apps.policy.predict"
_DEFAULT_TIMEOUT = 5.0

__all__ = ["NatsPolicyClient", "pack", "pack_image"]


class NatsPolicyClient(PolicyClient):
    """Policy client that communicates via NATS request/reply.

    Scalars are sent as msgpack request/reply. Images are published
    on separate subjects (one per camera) so each stays under NATS
    max_payload (1MB).

    Parameters
    ----------
    nats_client:
        A connected ``nats.aio.client.Client``.
    subject:
        NATS subject the policy service listens on.
    timeout:
        Request/reply timeout in seconds.
    """

    def __init__(
        self,
        nats_client: nats.aio.client.Client,
        *,
        subject: str = _DEFAULT_SUBJECT,
        timeout: float = _DEFAULT_TIMEOUT,
    ) -> None:
        self._nc = nats_client
        self._subject = subject
        self._timeout = timeout
        self._motion_group_ids: list[str] = []

    async def connect(self, motion_group_ids: list[str]) -> None:
        """Store the motion group IDs.  No extra connection needed."""
        self._motion_group_ids = list(motion_group_ids)
        logger.info(
            "NatsPolicyClient ready (%d groups) on subject %r",
            len(motion_group_ids),
            self._subject,
        )

    async def get_actions(
        self,
        states: dict[str, RobotState],
        schema: PolicySchema,
        images: dict[str, NDArray[Any]] | None = None,
        io_values: dict[str, object] | None = None,
    ) -> ActionChunk:
        """Build flat obs, publish images separately, request/reply for actions."""
        # Build flat scalar observation
        flat_obs: dict[str, Any] = schema.build_observation(states, io_values)

        # Publish images on separate subjects
        image_names: list[str] = []
        if images:
            for cam_name, frame in images.items():
                img_subject = f"{self._subject}.images.{cam_name}"
                await self._nc.publish(img_subject, pack_image(frame))
                image_names.append(cam_name)

        if image_names:
            flat_obs["__images__"] = image_names

        # Request/reply with scalars only (well within 1MB)
        payload = pack(flat_obs)
        msg = await self._nc.request(self._subject, payload, timeout=self._timeout)
        raw = _unpack_response(msg.data)
        return self._parse_response(raw, schema)

    async def close(self) -> None:
        """No-op — caller owns the NATS connection lifecycle."""
        logger.info("NatsPolicyClient closed (NATS connection still owned by caller)")

    def _parse_response(self, raw: dict[str, Any], schema: PolicySchema) -> ActionChunk:
        """Parse the reply into an ActionChunk.

        If the response has 'joints', parse as structured PolicyResponse.
        Otherwise treat the entire dict as flat features and convert via schema.
        """
        if "joints" in raw and raw["joints"] is not None:
            resp = PolicyResponse.model_validate(raw)
            return ActionChunk(joints=resp.joints, ios=resp.ios, dt_ms=resp.dt_ms)

        # Flat feature dict — parse via schema
        joints, ios = schema.parse_action(raw)
        if joints:
            return ActionChunk(joints=joints, ios=ios)

        msg = "Policy returned no joints or features"
        raise RuntimeError(msg)


def _unpack_response(data: bytes) -> dict[str, Any]:
    """Unpack a response (always msgpack scalars)."""
    import msgpack  # noqa: PLC0415

    return msgpack.unpackb(data)
