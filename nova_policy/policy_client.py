"""PolicyClient protocol and built-in implementations."""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Any, Protocol

from nova_policy.types import ActionChunk, PolicyDone

if TYPE_CHECKING:
    from nova_policy.types import PolicyResult

logger = logging.getLogger(__name__)


class PolicyClient(Protocol):
    """Protocol for policy action sources.

    Implementations connect to a policy (local model, NATS, ZMQ, etc.)
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
