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

import numpy as np

from policy.nats_wire import pack, pack_image
from policy.types import ActionChunk, PolicyResponse

if TYPE_CHECKING:
    import nats.aio.client

logger = logging.getLogger(__name__)

_DEFAULT_SUBJECT = "nova.v2.cells.cell.apps.policy.predict"
_DEFAULT_TIMEOUT = 5.0

__all__ = ["NatsPolicyClient", "pack", "pack_image"]


class NatsPolicyClient:
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

    async def get_actions(self, obs: dict[str, Any]) -> ActionChunk | dict[str, float]:
        """Send observation (images published separately), await policy reply."""
        # Separate images from scalars
        scalars: dict[str, Any] = {}
        image_names: list[str] = []
        for key, value in obs.items():
            if isinstance(value, np.ndarray):
                # Publish image on its own subject
                img_subject = f"{self._subject}.images.{key}"
                await self._nc.publish(img_subject, pack_image(value))
                image_names.append(key)
            else:
                scalars[key] = value

        # Include image names so the policy knows which cameras are active
        if image_names:
            scalars["__images__"] = image_names

        # Request/reply with scalars only (well within 1MB)
        payload = pack(scalars)
        msg = await self._nc.request(self._subject, payload, timeout=self._timeout)
        raw = _unpack_response(msg.data)
        return self._parse_response(raw)

    async def close(self) -> None:
        """No-op — caller owns the NATS connection lifecycle."""
        logger.info("NatsPolicyClient closed (NATS connection still owned by caller)")

    @staticmethod
    def _parse_response(raw: dict[str, Any]) -> ActionChunk | dict[str, float]:
        """Parse the reply into a typed result."""
        resp = PolicyResponse.model_validate(raw)

        if resp.features and not resp.joints:
            return resp.features
        if resp.joints:
            return ActionChunk(joints=resp.joints, ios=resp.ios, dt_ms=resp.dt_ms)

        msg = "Policy returned no joints or features"
        raise RuntimeError(msg)


def _unpack_response(data: bytes) -> dict[str, Any]:
    """Unpack a response (always msgpack scalars)."""
    import msgpack  # noqa: PLC0415

    return msgpack.unpackb(data)
