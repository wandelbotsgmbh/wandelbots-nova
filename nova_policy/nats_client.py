"""NATS request/reply policy client.

Uses NATS for app-to-app policy inference on the Nova platform.
The policy service subscribes to a subject and replies with actions.

Protocol (JSON over NATS request/reply):

    Request (observation):
        {"joints": [...], "pose": [...], "motion_group_id": "0@ur10e"}
        — or flat feature dict when using FeatureMap mode —

    Reply (PolicyResponse JSON):
        {"joints": {"0@ur10e": [[...]]}, "dt_ms": 33.0}
        {"done": true}
        {"waiting": true}
        {"features": {"left_joint_1.pos": 0.1, ...}}
"""

from __future__ import annotations

import contextlib
import json
import logging
from typing import TYPE_CHECKING, Any

from nova_policy.types import ActionChunk, PolicyDone, PolicyResponse, PolicyWaiting

if TYPE_CHECKING:
    import nats.aio.client

logger = logging.getLogger(__name__)

_DEFAULT_SUBJECT = "nova.v2.cells.cell.apps.policy.predict"
_DEFAULT_TIMEOUT = 5.0


class NatsPolicyClient:
    """Policy client that communicates via NATS request/reply.

    The policy service subscribes to ``subject`` and responds with
    :class:`~nova_policy.types.PolicyResponse` JSON.

    Parameters
    ----------
    nats_client:
        A connected ``nats.aio.client.Client``.  The caller owns the
        lifecycle (connect / drain) — this class only publishes and
        requests on it.
    subject:
        NATS subject the policy service listens on.
        Default ``"nova.v2.cells.cell.apps.policy.predict"``.
        Convention: ``{instance}.v2.cells.{cell}.apps.{app}.{action}``
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
        """Store the motion group IDs.  No extra connection needed — NATS is already connected."""
        self._motion_group_ids = list(motion_group_ids)
        logger.info(
            "NatsPolicyClient ready (%d groups) on subject %r",
            len(motion_group_ids),
            self._subject,
        )

    async def get_actions(self, obs: dict[str, Any]) -> ActionChunk | PolicyDone | PolicyWaiting | dict[str, float]:
        """Publish observation, await policy reply."""
        payload = json.dumps(self._build_request(obs)).encode()
        msg = await self._nc.request(self._subject, payload, timeout=self._timeout)
        raw = json.loads(msg.data.decode())
        return self._parse_response(raw)

    async def notify_stopped(self, reason: str) -> None:
        """Best-effort publish of a stop notification."""
        with contextlib.suppress(Exception):
            data = json.dumps({"executor_stopped": True, "reason": reason}).encode()
            await self._nc.publish(self._subject, data)

    async def close(self) -> None:
        """No-op — caller owns the NATS connection lifecycle."""
        logger.info("NatsPolicyClient closed (NATS connection still owned by caller)")

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _build_request(self, obs: dict[str, Any]) -> dict[str, Any]:
        """Package the observation dict for NATS."""
        # FeatureMap mode: obs is already a flat dict
        if self._motion_group_ids and not any(
            mg_id in obs for mg_id in self._motion_group_ids
        ):
            return obs

        # Structured mode: serialize per-motion-group states
        result: dict[str, Any] = {}
        for mg_id in self._motion_group_ids:
            state = obs.get(mg_id)
            if state is None:
                continue
            if hasattr(state, "joints"):
                entry: dict[str, Any] = {"joints": list(state.joints), "motion_group_id": mg_id}
                if hasattr(state, "pose") and state.pose is not None:
                    entry["pose"] = list(state.pose.position) + list(state.pose.orientation)
                result[mg_id] = entry
            elif isinstance(state, dict):
                result[mg_id] = {"motion_group_id": mg_id, **state}
        return result

    @staticmethod
    def _parse_response(raw: dict[str, Any]) -> ActionChunk | PolicyDone | PolicyWaiting | dict[str, float]:
        """Parse the JSON reply into a typed result."""
        resp = PolicyResponse.model_validate(raw)

        if resp.done:
            return PolicyDone()
        if resp.waiting:
            return PolicyWaiting()
        if resp.features and not resp.joints:
            return resp.features
        if resp.joints:
            return ActionChunk(joints=resp.joints, ios=resp.ios, dt_ms=resp.dt_ms)
        return PolicyWaiting()
