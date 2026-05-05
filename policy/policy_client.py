"""PolicyClient protocol and built-in implementations."""

from __future__ import annotations

import json
import logging
from typing import Any, Protocol

import websockets

from policy.types import ActionChunk, PolicyResponse

logger = logging.getLogger(__name__)


class PolicyClient(Protocol):
    """Protocol for policy action sources.

    A policy is a pure function: obs → actions.
    It never signals "done" — episode termination is an executor concern.
    """

    async def connect(self, motion_group_ids: list[str]) -> None:
        """Establish connection to the policy service."""
        ...

    async def get_actions(self, obs: dict[str, Any]) -> ActionChunk | dict[str, float]:
        """Send observation, receive action chunk.

        Returns:
            ActionChunk — joint targets to execute.
            dict[str, float] — flat feature dict (FeatureMap mode).
        """
        ...

    async def close(self) -> None:
        """Close the connection."""
        ...


class WebSocketPolicyClient:
    """Policy client that communicates via WebSocket.

    Kept for local development where WebSocket is reachable directly.
    On the Nova platform, prefer NatsPolicyClient.
    """

    def __init__(self, url: str) -> None:
        self._url = url
        self._connections: dict[str, websockets.WebSocketClientProtocol] = {}
        self._motion_group_ids: list[str] = []

    async def connect(self, motion_group_ids: list[str]) -> None:
        self._motion_group_ids = motion_group_ids
        for mg_id in motion_group_ids:
            ws = await websockets.connect(self._url)
            self._connections[mg_id] = ws
        logger.info(
            "WebSocketPolicyClient connected (%d groups) to %s", len(motion_group_ids), self._url
        )

    async def get_actions(self, obs: dict[str, Any]) -> ActionChunk | dict[str, float]:
        all_joints: dict[str, list[list[float]]] = {}
        all_ios: dict[str, dict[str, bool | int | float | str]] = {}
        all_features: dict[str, float] = {}
        dt_ms = 0.0

        for mg_id, ws in self._connections.items():
            payload = self._serialize_observation(mg_id, obs.get(mg_id))
            if payload is None:
                continue

            await ws.send(json.dumps(payload))
            raw = json.loads(await ws.recv())
            resp = PolicyResponse.model_validate(raw)

            if resp.joints:
                all_joints.update(resp.joints)
            if resp.ios:
                all_ios.update(resp.ios)
            if resp.features:
                all_features.update(resp.features)
            dt_ms = resp.dt_ms

        if all_features and not all_joints:
            return all_features

        if not all_joints:
            msg = "Policy returned no joints"
            raise RuntimeError(msg)

        return ActionChunk(joints=all_joints, ios=all_ios or None, dt_ms=dt_ms)

    async def close(self) -> None:
        for ws in self._connections.values():
            await ws.close()
        self._connections.clear()
        logger.info("WebSocketPolicyClient closed")

    @staticmethod
    def _serialize_observation(mg_id: str, state: object) -> dict[str, Any] | None:
        if state is None:
            return None

        if hasattr(state, "joints"):
            payload: dict[str, Any] = {"joints": list(state.joints), "motion_group_id": mg_id}
            if hasattr(state, "pose") and state.pose is not None:
                payload["pose"] = list(state.pose.position) + list(state.pose.orientation)
            return payload

        if isinstance(state, dict):
            return {"motion_group_id": mg_id, **state}

        return None


class CallbackPolicyClient:
    """Policy client that calls a local async function.

    The function receives observations and must return:
    - An ActionChunk
    - A dict with "joints" key (converted to ActionChunk)
    - A flat feature dict (FeatureMap mode)
    """

    def __init__(self, fn: object) -> None:
        self._fn = fn

    async def connect(self, motion_group_ids: list[str]) -> None:
        pass

    async def get_actions(self, obs: dict[str, Any]) -> ActionChunk | dict[str, float]:
        result = await self._fn(obs)  # type: ignore[operator]
        if isinstance(result, ActionChunk):
            return result
        if isinstance(result, dict):
            if "joints" in result:
                return ActionChunk.from_dict(result)
            # Flat feature dict
            return result  # type: ignore[return-value]
        msg = f"Policy callback must return ActionChunk or dict, got {type(result).__name__}"
        raise TypeError(msg)

    async def close(self) -> None:
        pass
