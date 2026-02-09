"""Async action support for executing arbitrary async callables during trajectory execution.

This module provides the AsyncAction type and supporting infrastructure for executing
user-defined async functions at specific locations along a robot trajectory. Actions
can run in parallel with robot motion (default) or block motion until complete.

Key concepts:
    - **AsyncAction**: An action that triggers an async callable at a trajectory location.
    - **ActionRegistry**: Maps string names to async callables for serialization support.
    - **AsyncActionResult**: Captures execution results including timing and error info.

Example usage:
    ```python
    from nova.actions import async_action, register_async_action
    from nova.actions.async_action import ActionExecutionContext

    # Define an async action handler
    async def log_position(ctx: ActionExecutionContext):
        print(f"Robot at location {ctx.trigger_location}: {ctx.current_state.pose}")

    # Register it
    register_async_action("log_position", log_position)

    # Use in trajectory
    actions = [
        ptp(pose1),
        async_action("log_position"),  # Fires after ptp completes
        lin(pose2),
        async_action("log_position", blocking=True),  # Pauses motion during execution
    ]
    ```

Future enhancements:
    - TODO: Add sync function support via asyncio.to_thread() wrapper
    - TODO: Add action priority/ordering within same location
    - TODO: Add action cancellation on trajectory abort
"""

from __future__ import annotations

import asyncio
import logging
from dataclasses import dataclass, field
from datetime import datetime
from enum import StrEnum, auto
from typing import Any, Awaitable, Callable, ClassVar, Literal

import pydantic

from nova.actions.base import Action
from nova.types.state import RobotState

logger = logging.getLogger(__name__)


@dataclass
class ActionExecutionContext:
    """Context passed to async action handlers during execution.

    Provides information about the robot state and trajectory location when
    the action was triggered.

    Attributes:
        trigger_location: The path parameter where the action was triggered (float).
        current_state: The robot state (pose, joints, tcp) at trigger time.
        motion_group_id: Identifier of the motion group executing the trajectory.
        action_name: The registered name of the action being executed.
        action_args: Positional arguments passed to the action.
        action_kwargs: Keyword arguments passed to the action.
    """

    trigger_location: float
    current_state: RobotState
    motion_group_id: str
    action_name: str
    action_args: tuple[Any, ...] = ()
    action_kwargs: dict[str, Any] = field(default_factory=dict)


# Type alias for async action handlers
AsyncActionHandler = Callable[[ActionExecutionContext], Awaitable[Any]]


class ActionRegistry:
    """Registry mapping action names to async callable handlers.

    Provides a centralized store for async action handlers that can be
    referenced by name in AsyncAction instances. This enables serialization
    of action sequences while keeping the actual callable logic separate.

    Example:
        ```python
        registry = ActionRegistry()
        registry.register("my_action", my_async_function)
        handler = registry.get("my_action")
        await handler(context)
        ```

    Note:
        A global default registry is provided via `get_default_registry()`.
        Most users should use the module-level `register_async_action()` function.
    """

    def __init__(self):
        self._handlers: dict[str, AsyncActionHandler] = {}

    def register(self, name: str, handler: AsyncActionHandler) -> None:
        """Register an async handler with the given name.

        Args:
            name: Unique identifier for the action.
            handler: Async callable that accepts ActionExecutionContext.

        Raises:
            ValueError: If name is already registered.
            TypeError: If handler is not a coroutine function.

        Note:
            # TODO: Future enhancement - support sync functions by auto-wrapping
            # with asyncio.to_thread(). For now, only async functions are accepted.
        """
        if name in self._handlers:
            raise ValueError(f"Action '{name}' is already registered")

        if not asyncio.iscoroutinefunction(handler):
            # TODO: Add sync support via:
            # async def async_wrapper(ctx: ActionExecutionContext):
            #     return await asyncio.to_thread(handler, ctx)
            # self._handlers[name] = async_wrapper
            raise TypeError(
                f"Handler for '{name}' must be an async function (coroutine function). "
                "Sync function support is planned for a future release."
            )

        self._handlers[name] = handler
        logger.debug(f"Registered async action: {name}")

    def unregister(self, name: str) -> None:
        """Remove a registered handler.

        Args:
            name: The action name to unregister.

        Raises:
            KeyError: If name is not registered.
        """
        if name not in self._handlers:
            raise KeyError(f"Action '{name}' is not registered")
        del self._handlers[name]
        logger.debug(f"Unregistered async action: {name}")

    def get(self, name: str) -> AsyncActionHandler:
        """Get handler by name.

        Args:
            name: The registered action name.

        Returns:
            The async callable handler.

        Raises:
            KeyError: If name is not registered.
        """
        if name not in self._handlers:
            raise KeyError(
                f"Action '{name}' is not registered. "
                f"Available actions: {list(self._handlers.keys())}"
            )
        return self._handlers[name]

    def is_registered(self, name: str) -> bool:
        """Check if an action name is registered."""
        return name in self._handlers

    def list_actions(self) -> list[str]:
        """Return list of all registered action names."""
        return list(self._handlers.keys())

    def clear(self) -> None:
        """Remove all registered handlers. Mainly useful for testing."""
        self._handlers.clear()


# Global default registry
_default_registry: ActionRegistry | None = None


def get_default_registry() -> ActionRegistry:
    """Get the global default action registry.

    Creates the registry on first access (lazy initialization).
    """
    global _default_registry
    if _default_registry is None:
        _default_registry = ActionRegistry()
    return _default_registry


def register_async_action(name: str, handler: AsyncActionHandler) -> None:
    """Register an async action handler in the default registry.

    This is the primary API for registering action handlers.

    Args:
        name: Unique identifier for the action.
        handler: Async callable that accepts ActionExecutionContext.

    Example:
        ```python
        async def my_handler(ctx: ActionExecutionContext):
            print(f"Triggered at {ctx.trigger_location}")

        register_async_action("my_handler", my_handler)
        ```
    """
    get_default_registry().register(name, handler)


def unregister_async_action(name: str) -> None:
    """Remove an async action handler from the default registry."""
    get_default_registry().unregister(name)


class ErrorHandlingMode(StrEnum):
    """How errors in async actions should be handled.

    Attributes:
        RAISE: Stop execution and propagate the error immediately.
        COLLECT: Store error in results and continue execution.
        CALLBACK: Call user-provided error handler, then continue.
    """

    RAISE = auto()
    COLLECT = auto()
    CALLBACK = auto()


class AsyncAction(Action):
    """An action that executes an async callable at a specific trajectory location.

    AsyncAction triggers user-defined async functions during trajectory execution.
    The callable is looked up by name in the action registry at execution time.

    Attributes:
        type: Literal type discriminator for serialization.
        action_name: Name of the registered async handler to invoke.
        args: Positional arguments to pass to the handler (via context).
        kwargs: Keyword arguments to pass to the handler (via context).
        blocking: If True, pause robot motion during action execution.
        timeout: Optional timeout in seconds. None means no timeout.

    Example:
        ```python
        # Non-blocking action (runs in parallel with motion)
        async_action("log_data")

        # Blocking action (pauses motion)
        async_action("take_photo", blocking=True, timeout=5.0)

        # With arguments
        async_action("send_notification", "completed", level="info")
        ```
    """

    type: Literal["AsyncAction"] = "AsyncAction"
    action_name: str
    args: tuple[Any, ...] = ()
    kwargs: dict[str, Any] = pydantic.Field(default_factory=dict)
    blocking: bool = False
    timeout: float | None = None

    # Reference to registry (not serialized) - uses different name to avoid collision with Action._registry
    _action_registry: ClassVar[ActionRegistry | None] = None

    @classmethod
    def set_registry(cls, registry: ActionRegistry) -> None:
        """Set the registry used to resolve action names.

        If not set, the default global registry is used.
        """
        cls._action_registry = registry

    @classmethod
    def get_registry(cls) -> ActionRegistry:
        """Get the registry used to resolve action names."""
        return cls._action_registry or get_default_registry()

    def get_handler(self) -> AsyncActionHandler:
        """Get the async handler for this action.

        Returns:
            The registered async callable.

        Raises:
            KeyError: If action_name is not registered.
        """
        return self.get_registry().get(self.action_name)

    def is_motion(self) -> bool:
        """AsyncAction is not a motion primitive."""
        return False

    def to_api_model(self) -> dict[str, Any]:
        """Serialize to dict form.

        Note: AsyncActions are executed client-side and not sent to the
        robot controller, so this just returns a dict representation.
        """
        return self.model_dump(exclude={"metas"})


@dataclass
class AsyncActionResult:
    """Result of an async action execution.

    Captures timing, location, and error information for completed actions.
    Used for debugging, logging, and error handling.

    Attributes:
        action: The AsyncAction that was executed.
        trigger_location: Path parameter where action was triggered.
        completion_location: Path parameter when action completed (None if not tracked).
        started_at: Timestamp when execution began.
        completed_at: Timestamp when execution finished.
        duration_seconds: Execution duration.
        return_value: Value returned by the handler (if any).
        error: Exception if execution failed, None otherwise.
        was_blocking: Whether the action blocked robot motion.
        timed_out: Whether the action was terminated due to timeout.
    """

    action: AsyncAction
    trigger_location: float
    started_at: datetime
    completed_at: datetime
    completion_location: float | None = None
    return_value: Any = None
    error: Exception | None = None
    was_blocking: bool = False
    timed_out: bool = False

    @property
    def duration_seconds(self) -> float:
        """Calculate execution duration in seconds."""
        return (self.completed_at - self.started_at).total_seconds()

    @property
    def succeeded(self) -> bool:
        """Whether the action completed successfully."""
        return self.error is None and not self.timed_out

    def to_dict(self) -> dict[str, Any]:
        """Convert to dictionary for logging/serialization."""
        return {
            "action_name": self.action.action_name,
            "trigger_location": self.trigger_location,
            "completion_location": self.completion_location,
            "started_at": self.started_at.isoformat(),
            "completed_at": self.completed_at.isoformat(),
            "duration_seconds": self.duration_seconds,
            "succeeded": self.succeeded,
            "was_blocking": self.was_blocking,
            "timed_out": self.timed_out,
            "error": str(self.error) if self.error else None,
        }


def async_action(
    name: str, *args: Any, blocking: bool = False, timeout: float | None = None, **kwargs: Any
) -> AsyncAction:
    """Create an AsyncAction that executes a registered async handler.

    Factory function for creating AsyncAction instances with a convenient API.

    Args:
        name: Name of the registered async handler to invoke.
        *args: Positional arguments passed to handler via context.
        blocking: If True, pause robot motion during execution. Default False.
        timeout: Optional timeout in seconds. None means no timeout.
        **kwargs: Keyword arguments passed to handler via context.

    Returns:
        AsyncAction instance ready to be included in an action sequence.

    Example:
        ```python
        # Simple async action (runs parallel to motion)
        async_action("log_position")

        # Blocking action with timeout
        async_action("capture_image", blocking=True, timeout=2.0)

        # With arguments
        async_action("send_data", target="server", value=42)
        ```

    Raises:
        KeyError: At execution time if name is not registered.
    """
    return AsyncAction(
        action_name=name, args=args, kwargs=kwargs, blocking=blocking, timeout=timeout
    )
