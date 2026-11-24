"""Strict policy-client interface and local callback adapter."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from collections.abc import Callable

    from numpy.typing import NDArray

    from nova.types import RobotState
    from novapolicy.schema import PolicySchema
    from novapolicy.types import ActionChunk


class PolicyClient(ABC):
    """Base class for every policy action source used by ``PolicyExecutor``.

    Lifecycle and optional continuous-execution hooks have explicit no-op
    defaults. Concrete clients only need to implement :meth:`get_actions` and
    override the hooks they support.
    """

    async def connect(self, motion_group_ids: list[str]) -> None:  # ruff: ignore[empty-method-without-abstract-decorator]
        """Establish a policy-service connection."""

    async def validate_schema(self, schema: PolicySchema) -> None:  # ruff: ignore[empty-method-without-abstract-decorator]
        """Raise ``ValueError`` when the schema cannot satisfy the policy."""

    async def prepare(  # ruff: ignore[empty-method-without-abstract-decorator]
        self,
        states: dict[str, RobotState],
        schema: PolicySchema,
        images: dict[str, NDArray[Any]] | None = None,
        io_values: dict[str, object] | None = None,
    ) -> None:
        """Perform optional setup before the execution timeout starts."""

    @abstractmethod
    async def get_actions(
        self,
        states: dict[str, RobotState],
        schema: PolicySchema,
        images: dict[str, NDArray[Any]] | None = None,
        io_values: dict[str, object] | None = None,
    ) -> ActionChunk:
        """Return one action chunk for the current observation."""

    async def close(self) -> None:  # ruff: ignore[empty-method-without-abstract-decorator]
        """Release policy-service resources."""

    def synchronize_action_timestep(self, timestep: int) -> None:  # ruff: ignore[empty-method-without-abstract-decorator]
        """Synchronize an asynchronous queue to the controller timestep."""

    @property
    def requires_first_waypoint_bridge(self) -> bool:
        """Whether continuous execution needs one measured-state bridge."""
        return False

    @property
    def n_action_steps(self) -> int | None:
        """Steps to execute per chunk when the policy declares its own horizon.

        ``None`` leaves the choice to the executor. Clients that read a
        checkpoint report its value here so the caller does not have to copy it.
        """
        return None

    @property
    def rtc(self) -> object | None:
        """Model-side RTC configuration, or ``None`` when RTC is disabled."""
        return None


class CallbackPolicyClient(PolicyClient):
    """Explicit adapter for a local asynchronous policy callback."""

    def __init__(self, fn: Callable[..., Any]) -> None:
        self._fn = fn

    async def get_actions(
        self,
        states: dict[str, RobotState],
        schema: PolicySchema,
        images: dict[str, NDArray[Any]] | None = None,
        io_values: dict[str, object] | None = None,
    ) -> ActionChunk:
        observation: dict[str, Any] = await schema.build_observation(states, io_values)
        if images:
            observation.update(images)
        return await self._fn(observation)
