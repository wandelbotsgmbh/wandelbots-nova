"""PolicyClient protocol and built-in implementations."""

from __future__ import annotations

import logging
from typing import Any, Protocol

from policy.types import ActionChunk

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
