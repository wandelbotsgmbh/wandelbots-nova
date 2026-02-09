"""Unit tests for async action functionality."""

import asyncio
from datetime import datetime

import pytest

from nova.actions.async_action import (
    ActionExecutionContext,
    ActionRegistry,
    AsyncAction,
    AsyncActionResult,
    ErrorHandlingMode,
    async_action,
    get_default_registry,
    register_async_action,
    unregister_async_action,
)
from nova.actions.container import ActionLocation, CombinedActions
from nova.actions.motions import linear
from nova.cell.movement_controller.async_action_executor import AsyncActionExecutor
from nova.exceptions import AsyncActionError
from nova.types import Pose
from nova.types.state import RobotState

# ============================================================================
# ActionRegistry Tests
# ============================================================================


class TestActionRegistry:
    """Tests for ActionRegistry class."""

    def test_register_and_get(self):
        """Test registering and retrieving async handlers."""
        registry = ActionRegistry()

        async def handler(ctx: ActionExecutionContext):
            return "result"

        registry.register("test_action", handler)
        assert registry.is_registered("test_action")
        assert registry.get("test_action") is handler

    def test_register_duplicate_raises(self):
        """Test that registering duplicate name raises ValueError."""
        registry = ActionRegistry()

        async def handler(ctx: ActionExecutionContext):
            pass

        registry.register("action", handler)
        with pytest.raises(ValueError, match="already registered"):
            registry.register("action", handler)

    def test_register_sync_function_raises(self):
        """Test that registering sync function raises TypeError."""
        registry = ActionRegistry()

        def sync_handler(ctx: ActionExecutionContext):
            pass

        with pytest.raises(TypeError, match="must be an async function"):
            registry.register("action", sync_handler)

    def test_get_unregistered_raises(self):
        """Test that getting unregistered action raises KeyError."""
        registry = ActionRegistry()
        with pytest.raises(KeyError, match="not registered"):
            registry.get("nonexistent")

    def test_unregister(self):
        """Test unregistering handlers."""
        registry = ActionRegistry()

        async def handler(ctx: ActionExecutionContext):
            pass

        registry.register("action", handler)
        registry.unregister("action")
        assert not registry.is_registered("action")

    def test_unregister_nonexistent_raises(self):
        """Test unregistering nonexistent action raises KeyError."""
        registry = ActionRegistry()
        with pytest.raises(KeyError, match="not registered"):
            registry.unregister("nonexistent")

    def test_list_actions(self):
        """Test listing registered actions."""
        registry = ActionRegistry()

        async def h1(ctx):
            pass

        async def h2(ctx):
            pass

        registry.register("a", h1)
        registry.register("b", h2)
        assert set(registry.list_actions()) == {"a", "b"}

    def test_clear(self):
        """Test clearing all handlers."""
        registry = ActionRegistry()

        async def handler(ctx):
            pass

        registry.register("action", handler)
        registry.clear()
        assert registry.list_actions() == []


class TestGlobalRegistry:
    """Tests for global registry functions."""

    def teardown_method(self):
        """Clean up global registry after each test."""
        get_default_registry().clear()

    def test_register_and_unregister(self):
        """Test module-level register/unregister functions."""

        async def handler(ctx: ActionExecutionContext):
            pass

        register_async_action("global_action", handler)
        assert get_default_registry().is_registered("global_action")

        unregister_async_action("global_action")
        assert not get_default_registry().is_registered("global_action")


# ============================================================================
# AsyncAction Tests
# ============================================================================


class TestAsyncAction:
    """Tests for AsyncAction class."""

    def teardown_method(self):
        """Clean up global registry after each test."""
        get_default_registry().clear()

    def test_is_motion_returns_false(self):
        """Test that AsyncAction.is_motion() returns False."""
        action = AsyncAction(action_name="test")
        assert action.is_motion() is False

    def test_to_api_model(self):
        """Test serialization to dict."""
        action = AsyncAction(
            action_name="test", args=(1, 2), kwargs={"key": "value"}, blocking=True, timeout=5.0
        )
        result = action.to_api_model()
        assert result["action_name"] == "test"
        assert result["args"] == (1, 2)
        assert result["kwargs"] == {"key": "value"}
        assert result["blocking"] is True
        assert result["timeout"] == 5.0

    def test_get_handler(self):
        """Test getting handler from action."""

        async def handler(ctx):
            pass

        register_async_action("my_action", handler)
        action = AsyncAction(action_name="my_action")
        assert action.get_handler() is handler

    def test_get_handler_unregistered_raises(self):
        """Test getting handler for unregistered action raises."""
        action = AsyncAction(action_name="nonexistent")
        with pytest.raises(KeyError):
            action.get_handler()


class TestAsyncActionFactory:
    """Tests for async_action factory function."""

    def test_basic_creation(self):
        """Test basic action creation."""
        action = async_action("test")
        assert action.action_name == "test"
        assert action.blocking is False
        assert action.timeout is None
        assert action.args == ()
        assert action.kwargs == {}

    def test_with_args(self):
        """Test creation with positional args."""
        action = async_action("test", 1, 2, 3)
        assert action.args == (1, 2, 3)

    def test_with_kwargs(self):
        """Test creation with keyword args."""
        action = async_action("test", key="value", num=42)
        assert action.kwargs == {"key": "value", "num": 42}

    def test_blocking_and_timeout(self):
        """Test creation with blocking and timeout."""
        action = async_action("test", blocking=True, timeout=10.0)
        assert action.blocking is True
        assert action.timeout == 10.0


# ============================================================================
# AsyncActionResult Tests
# ============================================================================


class TestAsyncActionResult:
    """Tests for AsyncActionResult class."""

    def test_duration_calculation(self):
        """Test duration_seconds property."""
        started = datetime(2024, 1, 1, 12, 0, 0)
        completed = datetime(2024, 1, 1, 12, 0, 5)  # 5 seconds later

        result = AsyncActionResult(
            action=AsyncAction(action_name="test"),
            trigger_location=1.0,
            started_at=started,
            completed_at=completed,
        )
        assert result.duration_seconds == 5.0

    def test_succeeded_true(self):
        """Test succeeded property when no error."""
        result = AsyncActionResult(
            action=AsyncAction(action_name="test"),
            trigger_location=1.0,
            started_at=datetime.now(),
            completed_at=datetime.now(),
        )
        assert result.succeeded is True

    def test_succeeded_false_on_error(self):
        """Test succeeded property when error present."""
        result = AsyncActionResult(
            action=AsyncAction(action_name="test"),
            trigger_location=1.0,
            started_at=datetime.now(),
            completed_at=datetime.now(),
            error=ValueError("test error"),
        )
        assert result.succeeded is False

    def test_succeeded_false_on_timeout(self):
        """Test succeeded property when timed out."""
        result = AsyncActionResult(
            action=AsyncAction(action_name="test"),
            trigger_location=1.0,
            started_at=datetime.now(),
            completed_at=datetime.now(),
            timed_out=True,
        )
        assert result.succeeded is False

    def test_to_dict(self):
        """Test serialization to dict."""
        result = AsyncActionResult(
            action=AsyncAction(action_name="test"),
            trigger_location=1.5,
            completion_location=2.0,
            started_at=datetime(2024, 1, 1, 12, 0, 0),
            completed_at=datetime(2024, 1, 1, 12, 0, 1),
            was_blocking=True,
        )
        d = result.to_dict()
        assert d["action_name"] == "test"
        assert d["trigger_location"] == 1.5
        assert d["completion_location"] == 2.0
        assert d["was_blocking"] is True
        assert d["succeeded"] is True


# ============================================================================
# AsyncActionExecutor Tests
# ============================================================================


class TestAsyncActionExecutor:
    """Tests for AsyncActionExecutor class."""

    def teardown_method(self):
        """Clean up global registry after each test."""
        get_default_registry().clear()

    def _make_robot_state(self) -> RobotState:
        """Create a test robot state."""
        return RobotState(
            pose=Pose(position=(0, 0, 0), orientation=(0, 0, 0)),
            tcp="flange",
            joints=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        )

    @pytest.mark.asyncio
    async def test_trigger_at_location(self):
        """Test that actions are triggered when location is crossed."""
        call_log = []

        async def handler(ctx: ActionExecutionContext):
            call_log.append(ctx.trigger_location)

        register_async_action("log_action", handler)

        action = AsyncAction(action_name="log_action")
        action_location = ActionLocation(path_parameter=1.0, action=action)

        executor = AsyncActionExecutor(
            motion_group_id="test",
            async_actions=[action_location],
            error_mode=ErrorHandlingMode.COLLECT,
        )

        state = self._make_robot_state()

        # Before location - should not trigger
        await executor.check_and_trigger(0.5, state)
        assert len(call_log) == 0

        # At location - should trigger
        await executor.check_and_trigger(1.0, state)
        await executor.wait_for_all_actions()
        assert len(call_log) == 1
        assert call_log[0] == 1.0

        # After location - should not trigger again
        await executor.check_and_trigger(1.5, state)
        await executor.wait_for_all_actions()
        assert len(call_log) == 1

    @pytest.mark.asyncio
    async def test_multiple_actions_sorted_by_location(self):
        """Test actions at different locations are all triggered when location crosses them."""
        triggered_names = []

        async def handler(ctx: ActionExecutionContext):
            triggered_names.append(ctx.action_name)

        register_async_action("log", handler)

        actions = [
            ActionLocation(path_parameter=3.0, action=AsyncAction(action_name="log")),
            ActionLocation(path_parameter=1.0, action=AsyncAction(action_name="log")),
            ActionLocation(path_parameter=2.0, action=AsyncAction(action_name="log")),
        ]

        executor = AsyncActionExecutor(
            motion_group_id="test", async_actions=actions, error_mode=ErrorHandlingMode.COLLECT
        )

        state = self._make_robot_state()
        # Crossing location 3.5 should trigger all 3 actions (at 1.0, 2.0, and 3.0)
        await executor.check_and_trigger(3.5, state)
        await executor.wait_for_all_actions()

        # All actions should be triggered
        assert len(triggered_names) == 3
        # All pending actions should be marked as triggered
        assert not executor.has_pending_actions

    @pytest.mark.asyncio
    async def test_blocking_action_executes_inline(self):
        """Test that blocking actions are executed synchronously."""
        execution_order = []

        async def blocking_handler(ctx: ActionExecutionContext):
            execution_order.append("blocking")

        register_async_action("blocking", blocking_handler)

        action = AsyncAction(action_name="blocking", blocking=True)
        action_location = ActionLocation(path_parameter=1.0, action=action)

        executor = AsyncActionExecutor(motion_group_id="test", async_actions=[action_location])

        state = self._make_robot_state()
        await executor.check_and_trigger(1.0, state)

        # Blocking action should complete immediately
        assert execution_order == ["blocking"]
        assert len(executor.results) == 1
        assert executor.results[0].was_blocking is True

    @pytest.mark.asyncio
    async def test_timeout_handling(self):
        """Test action timeout handling."""

        async def slow_handler(ctx: ActionExecutionContext):
            await asyncio.sleep(10)

        register_async_action("slow", slow_handler)

        action = AsyncAction(action_name="slow", blocking=True, timeout=0.1)
        action_location = ActionLocation(path_parameter=1.0, action=action)

        executor = AsyncActionExecutor(
            motion_group_id="test",
            async_actions=[action_location],
            error_mode=ErrorHandlingMode.COLLECT,
        )

        state = self._make_robot_state()
        await executor.check_and_trigger(1.0, state)

        assert len(executor.results) == 1
        assert executor.results[0].timed_out is True
        assert executor.results[0].succeeded is False

    @pytest.mark.asyncio
    async def test_error_mode_raise(self):
        """Test RAISE error mode propagates exceptions."""

        async def failing_handler(ctx: ActionExecutionContext):
            raise ValueError("test error")

        register_async_action("failing", failing_handler)

        action = AsyncAction(action_name="failing", blocking=True)
        action_location = ActionLocation(path_parameter=1.0, action=action)

        executor = AsyncActionExecutor(
            motion_group_id="test",
            async_actions=[action_location],
            error_mode=ErrorHandlingMode.RAISE,
        )

        state = self._make_robot_state()
        with pytest.raises(AsyncActionError) as exc_info:
            await executor.check_and_trigger(1.0, state)

        assert "failing" in str(exc_info.value)
        assert exc_info.value.action_name == "failing"

    @pytest.mark.asyncio
    async def test_error_mode_collect(self):
        """Test COLLECT error mode stores errors in results."""

        async def failing_handler(ctx: ActionExecutionContext):
            raise ValueError("test error")

        register_async_action("failing", failing_handler)

        action = AsyncAction(action_name="failing", blocking=True)
        action_location = ActionLocation(path_parameter=1.0, action=action)

        executor = AsyncActionExecutor(
            motion_group_id="test",
            async_actions=[action_location],
            error_mode=ErrorHandlingMode.COLLECT,
        )

        state = self._make_robot_state()
        # Should not raise
        await executor.check_and_trigger(1.0, state)

        assert len(executor.results) == 1
        assert executor.results[0].error is not None
        assert "test error" in str(executor.results[0].error)

    @pytest.mark.asyncio
    async def test_error_mode_callback(self):
        """Test CALLBACK error mode calls error handler."""
        error_results = []

        async def error_handler(result: AsyncActionResult):
            error_results.append(result)

        async def failing_handler(ctx: ActionExecutionContext):
            raise ValueError("test error")

        register_async_action("failing", failing_handler)

        action = AsyncAction(action_name="failing", blocking=True)
        action_location = ActionLocation(path_parameter=1.0, action=action)

        executor = AsyncActionExecutor(
            motion_group_id="test",
            async_actions=[action_location],
            error_mode=ErrorHandlingMode.CALLBACK,
            error_callback=error_handler,
        )

        state = self._make_robot_state()
        await executor.check_and_trigger(1.0, state)

        assert len(error_results) == 1
        assert error_results[0].action.action_name == "failing"

    @pytest.mark.asyncio
    async def test_get_summary(self):
        """Test execution summary generation."""

        async def handler(ctx):
            pass

        register_async_action("action", handler)

        actions = [
            ActionLocation(path_parameter=1.0, action=AsyncAction(action_name="action")),
            ActionLocation(path_parameter=2.0, action=AsyncAction(action_name="action")),
        ]

        executor = AsyncActionExecutor(motion_group_id="test-group", async_actions=actions)

        state = self._make_robot_state()
        await executor.check_and_trigger(1.5, state)
        await executor.wait_for_all_actions()

        summary = executor.get_summary()
        assert summary["motion_group_id"] == "test-group"
        assert summary["total_actions"] == 2
        assert summary["triggered"] == 1
        assert summary["completed"] == 1
        assert summary["succeeded"] == 1

    @pytest.mark.asyncio
    async def test_cancel_all_actions(self):
        """Test cancelling running actions."""
        started = asyncio.Event()

        async def long_handler(ctx: ActionExecutionContext):
            started.set()
            await asyncio.sleep(100)

        register_async_action("long", long_handler)

        action = AsyncAction(action_name="long")  # Non-blocking
        action_location = ActionLocation(path_parameter=1.0, action=action)

        executor = AsyncActionExecutor(motion_group_id="test", async_actions=[action_location])

        state = self._make_robot_state()
        await executor.check_and_trigger(1.0, state)

        # Wait for action to start
        await asyncio.wait_for(started.wait(), timeout=1.0)
        assert executor.has_running_actions

        # Cancel
        await executor.cancel_all_actions()
        assert not executor.has_running_actions


# ============================================================================
# CombinedActions Integration Tests
# ============================================================================


class TestCombinedActionsAsyncIntegration:
    """Tests for AsyncAction integration with CombinedActions."""

    def _make_linear(self):
        """Create a test Linear motion using factory function."""
        return linear((100, 0, 0, 0, 0, 0))

    def test_async_action_in_container(self):
        """Test AsyncAction can be added to CombinedActions."""
        actions = CombinedActions(
            items=(self._make_linear(), AsyncAction(action_name="test"), self._make_linear())
        )
        assert len(actions) == 3

    def test_get_async_actions(self):
        """Test extracting async actions with locations."""
        actions = CombinedActions(
            items=(
                self._make_linear(),  # location 1
                AsyncAction(action_name="first"),  # at location 1
                self._make_linear(),  # location 2
                self._make_linear(),  # location 3
                AsyncAction(action_name="second"),  # at location 3
            )
        )

        async_actions = actions.get_async_actions()
        assert len(async_actions) == 2
        assert async_actions[0].path_parameter == 1.0
        assert async_actions[0].action.action_name == "first"  # type: ignore
        assert async_actions[1].path_parameter == 3.0
        assert async_actions[1].action.action_name == "second"  # type: ignore

    def test_motions_excludes_async_actions(self):
        """Test that motions property doesn't include async actions."""
        actions = CombinedActions(
            items=(self._make_linear(), AsyncAction(action_name="test"), self._make_linear())
        )
        assert len(actions.motions) == 2

    def test_to_set_io_excludes_async_actions(self):
        """Test that to_set_io doesn't include async actions."""
        actions = CombinedActions(items=(self._make_linear(), AsyncAction(action_name="test")))
        # Should not raise and should return empty (no WriteActions)
        io_list = actions.to_set_io()
        assert io_list == []
