"""PolicyClient protocol and built-in implementations."""

from __future__ import annotations

import contextlib
import json
import logging
from typing import TYPE_CHECKING, Any, Protocol

import websockets

from nova_policy.types import ActionChunk, PolicyDone, PolicyResponse, PolicyWaiting

if TYPE_CHECKING:
    from nova_policy.types import PolicyResult

logger = logging.getLogger(__name__)


class PolicyClient(Protocol):
    """Protocol for policy action sources.

    Implementations connect to a policy (local model, WebSocket, gRPC, etc.)
    and translate observations into action chunks.
    """

    async def connect(self, motion_group_ids: list[str]) -> None:
        """Establish connection to the policy."""
        ...

    async def get_actions(self, obs: dict[str, Any]) -> PolicyResult:
        """Send observation, receive policy response.

        Returns one of:
            ActionChunk — targets to execute.
            PolicyDone — episode is done, trigger on_reset.
            PolicyWaiting — not ready yet, hold position.
        """
        ...

    async def notify_stopped(self, reason: str) -> None:
        """Notify the policy that the executor stopped.

        Called when the executor stops for any reason (e-stop, safety guard,
        user stop, error). The policy can use this to clean up state.

        This is optional — implementations that don't need it can pass.
        """
        ...

    async def close(self) -> None:
        """Close the connection."""
        ...


class WebSocketPolicyClient:
    """Policy client that communicates via WebSocket.

    Protocol:
        Sends: {"joints": [...], "pose": [...], "motion_group_id": "..."}
        Receives: PolicyResponse JSON (see PolicyResponse model)
    """

    def __init__(self, url: str) -> None:
        self._url = url
        self._connections: dict[str, websockets.WebSocketClientProtocol] = {}
        self._motion_group_ids: list[str] = []

    async def connect(self, motion_group_ids: list[str]) -> None:
        """Open one WebSocket per motion group."""
        self._motion_group_ids = motion_group_ids
        for mg_id in motion_group_ids:
            ws = await websockets.connect(self._url)
            self._connections[mg_id] = ws
        logger.info(
            "WebSocketPolicyClient connected (%d groups) to %s", len(motion_group_ids), self._url
        )

    async def get_actions(self, obs: dict[str, Any]) -> PolicyResult:
        """Send observations and collect responses from all connections."""
        all_joints: dict[str, list[list[float]]] = {}
        all_ios: dict[str, dict[str, bool | int | float | str]] = {}
        all_features: dict[str, float] = {}
        dt_ms = 0.0
        any_waiting = False

        for mg_id, ws in self._connections.items():
            payload = self._serialize_observation(mg_id, obs.get(mg_id))
            if payload is None:
                continue

            await ws.send(json.dumps(payload))
            raw = json.loads(await ws.recv())
            resp = PolicyResponse.model_validate(raw)

            if resp.done:
                return PolicyDone()
            if resp.waiting:
                any_waiting = True
                continue
            if resp.joints:
                all_joints.update(resp.joints)
            if resp.ios:
                all_ios.update(resp.ios)
            if resp.features:
                all_features.update(resp.features)
            dt_ms = resp.dt_ms

        if any_waiting:
            return PolicyWaiting()

        # If we got flat features, return them for the executor to parse via FeatureMap
        if all_features and not all_joints:
            return all_features  # type: ignore[return-value]  # executor handles dict

        if not all_joints:
            return PolicyWaiting()

        return ActionChunk(joints=all_joints, ios=all_ios or None, dt_ms=dt_ms)

    async def notify_stopped(self, reason: str) -> None:
        """Send stop reason to policy via a final WebSocket message before closing."""
        for ws in self._connections.values():
            with contextlib.suppress(OSError, RuntimeError):
                await ws.send(json.dumps({"executor_stopped": True, "reason": reason}))

    async def close(self) -> None:
        """Close all WebSocket connections."""
        for ws in self._connections.values():
            await ws.close()
        self._connections.clear()
        logger.info("WebSocketPolicyClient closed")

    @staticmethod
    def _serialize_observation(mg_id: str, state: object) -> dict[str, Any] | None:
        """Serialize observation for one motion group."""
        if state is None:
            return None

        if hasattr(state, "joints"):
            payload: dict[str, Any] = {"joints": list(state.joints), "motion_group_id": mg_id}
            if hasattr(state, "pose") and state.pose is not None:
                payload["pose"] = list(state.pose.position) + list(state.pose.orientation)
            if hasattr(state, "tcp") and state.tcp is not None:
                payload["tcp"] = state.tcp
            return payload

        if isinstance(state, dict):
            return {"motion_group_id": mg_id, **state}

        return None


class CallbackPolicyClient:
    """Policy client that calls a local async function.

    The function receives observations and returns:
    - An ActionChunk → execute targets
    - None → episode done

    No special types needed — just return ActionChunk or None.
    """

    def __init__(self, fn: object) -> None:
        self._fn = fn

    async def connect(self, motion_group_ids: list[str]) -> None:
        pass

    async def get_actions(self, obs: dict[str, Any]) -> PolicyResult:
        result = await self._fn(obs)  # type: ignore[operator]
        if result is None:
            return PolicyDone()
        if isinstance(result, ActionChunk):
            return result
        if isinstance(result, dict):
            # Could be structured {"joints": ...} or flat {"left_joint_1.pos": ...}
            if "joints" in result:
                return ActionChunk.from_dict(result)
            # Flat feature dict — return as-is for executor to parse via FeatureMap
            return result  # type: ignore[return-value]
        return ActionChunk.from_dict(result)

    async def notify_stopped(self, reason: str) -> None:
        """No-op for local callbacks."""

    async def close(self) -> None:
        pass
