"""PolicyClient protocol and built-in implementations."""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Any, Protocol, runtime_checkable

from policy.types import ActionChunk

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from nova.types import RobotState
    from policy.schema import PolicySchema

logger = logging.getLogger(__name__)


@runtime_checkable
class PolicyClient(Protocol):
    """Protocol for policy action sources.

    A policy is a pure function: (robot states, images) → ActionChunk.
    It never signals “done” — episode termination is an executor concern.

    The executor owns the ``PolicySchema`` and passes it to ``get_actions()``
    on every call.
    """

    async def connect(self, motion_group_ids: list[str]) -> None:
        """Establish connection to the policy service."""
        ...

    async def validate_schema(self, schema: PolicySchema) -> None:
        """Validate that the schema satisfies the policy's requirements.

        Called by the executor after ``connect()`` and before the first
        inference call.  Implementations should raise ``ValueError`` if the
        schema is missing keys the policy expects.

        The default implementation is a no-op — override in clients that
        can introspect the server's expected inputs (e.g. GR00T's
        ``get_modality_config``).
        """
        ...

    async def get_actions(
        self,
        states: dict[str, RobotState],
        schema: PolicySchema,
        images: dict[str, NDArray[Any]] | None = None,
        io_values: dict[str, object] | None = None,
    ) -> ActionChunk:
        """Receive robot states + camera images, return action chunk."""
        ...

    async def close(self) -> None:
        """Close the connection."""
        ...


class CallbackPolicyClient:
    """Policy client that calls a local async function.

    The user function receives a flat feature dict and returns either:
    - A flat feature dict (keys matching the PolicySchema)
    - An ActionChunk directly
    - A dict with "joints" key
    """

    def __init__(self, fn: object) -> None:
        self._fn = fn

    async def connect(self, motion_group_ids: list[str]) -> None:
        """No-op for local callbacks."""

    async def validate_schema(self, schema: PolicySchema) -> None:
        """No-op — bare functions don't declare expected keys."""

    async def close(self) -> None:
        pass

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
