"""PolicyClient protocol and built-in implementations."""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Any

from policy.types import ActionChunk

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from nova.types import RobotState
    from policy.schema import PolicySchema

logger = logging.getLogger(__name__)


class PolicyClient:
    """Base class for policy action sources.

    A policy is a pure function: (robot states, images) → ActionChunk.
    It never signals "done" — episode termination is an executor concern.

    The executor owns the ``PolicySchema`` and passes it to ``get_actions()``
    on every call.
    """

    async def connect(self, motion_group_ids: list[str]) -> None:
        """Establish connection to the policy service."""

    async def get_actions(
        self,
        states: dict[str, RobotState],
        schema: PolicySchema,
        images: dict[str, NDArray[Any]] | None = None,
        io_values: dict[str, object] | None = None,
    ) -> ActionChunk:
        """Receive robot states + camera images, return action chunk.

        Args:
            states: Dict mapping motion group ID → current RobotState.
            schema: The executor's PolicySchema for obs/action translation.
            images: Dict mapping camera name → numpy array (H,W,3) or (T,H,W,3).
                    None if no cameras configured.
            io_values: Dict mapping hardware IO key → current value.
                       None if no IOs configured.

        Returns:
            ActionChunk with joint targets (and optional IOs) to execute.
        """
        msg = "Subclasses must implement get_actions"
        raise NotImplementedError(msg)

    async def close(self) -> None:
        """Close the connection."""


class CallbackPolicyClient(PolicyClient):
    """Policy client that calls a local async function.

    The user function receives a flat feature dict and returns either:
    - A flat feature dict (keys matching the PolicySchema)
    - An ActionChunk directly
    - A dict with "joints" key
    """

    def __init__(self, fn: object) -> None:
        self._fn = fn

    async def get_actions(
        self,
        states: dict[str, RobotState],
        schema: PolicySchema,
        images: dict[str, NDArray[Any]] | None = None,
        io_values: dict[str, object] | None = None,
    ) -> ActionChunk:
        obs: dict[str, Any] = await schema.build_observation(states, io_values)
        if images:
            obs.update(images)

        result = await self._fn(obs)  # type: ignore[operator]

        if isinstance(result, ActionChunk):
            return result
        if isinstance(result, dict):
            if "joints" in result or "tcp" in result:
                return ActionChunk(**result)
            joints, tcp_targets, ios = await schema.parse_action(result)
            if joints or tcp_targets:
                return ActionChunk(joints=joints, tcp=tcp_targets, ios=ios)
        msg = f"Policy must return ActionChunk or dict, got {type(result).__name__}"
        raise TypeError(msg)
