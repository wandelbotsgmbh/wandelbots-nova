"""Unit tests for async action functionality."""

import asyncio
from datetime import datetime

import pytest

from nova.actions.async_action import (
    ActionExecutionContext,
    ActionRegistry,
    AsyncAction,
    AsyncActionResult,
    AwaitAction,
    ErrorHandlingMode,
    WaitUntilAction,
    async_action,
    await_action,
    get_default_registry,
    register_async_action,
    unregister_async_action,
    wait_until,
)
from nova.actions.container import ActionLocation, CombinedActions
from nova.actions.execution_state import ExecutionState
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
        action = AsyncAction(action_id="a1", action_name="test")
        assert action.is_motion() is False

    def test_to_api_model(self):
        """Test serialization to dict."""
        action = AsyncAction(
            action_id="a1", action_name="test", args=(1, 2), kwargs={"key": "value"}
        )
        result = action.to_api_model()
        assert result["action_id"] == "a1"
        assert result["action_name"] == "test"
        assert result["args"] == (1, 2)
        assert result["kwargs"] == {"key": "value"}

    def test_get_handler(self):
        """Test getting handler from action."""

        async def handler(ctx):
            pass

        register_async_action("my_action", handler)
        action = AsyncAction(action_id="a1", action_name="my_action")
        assert action.get_handler() is handler

    def test_get_handler_unregistered_raises(self):
        """Test getting handler for unregistered action raises."""
        action = AsyncAction(action_id="a1", action_name="nonexistent")
        with pytest.raises(KeyError):
            action.get_handler()


class TestAsyncActionFactory:
    """Tests for async_action factory function."""

    def test_basic_creation(self):
        """Test basic action creation."""
        action = async_action("test", action_id="a1")
        assert action.action_name == "test"
        assert action.action_id == "a1"
        assert action.args == ()
        assert action.kwargs == {}

    def test_with_args(self):
        """Test creation with positional args."""
        action = async_action("test", 1, 2, 3, action_id="a1")
        assert action.args == (1, 2, 3)

    def test_with_kwargs(self):
        """Test creation with keyword args."""
        action = async_action("test", action_id="a1", key="value", num=42)
        assert action.kwargs == {"key": "value", "num": 42}


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
            action=AsyncAction(action_id="a1", action_name="test"),
            trigger_location=1.0,
            started_at=started,
            completed_at=completed,
        )
        assert result.duration_seconds == 5.0

    def test_succeeded_true(self):
        """Test succeeded property when no error."""
        result = AsyncActionResult(
            action=AsyncAction(action_id="a1", action_name="test"),
            trigger_location=1.0,
            started_at=datetime.now(),
            completed_at=datetime.now(),
        )
        assert result.succeeded is True

    def test_succeeded_false_on_error(self):
        """Test succeeded property when error present."""
        result = AsyncActionResult(
            action=AsyncAction(action_id="a1", action_name="test"),
            trigger_location=1.0,
            started_at=datetime.now(),
            completed_at=datetime.now(),
            error=ValueError("test error"),
        )
        assert result.succeeded is False

    def test_succeeded_false_on_timeout(self):
        """Test succeeded property when timed out."""
        result = AsyncActionResult(
            action=AsyncAction(action_id="a1", action_name="test"),
            trigger_location=1.0,
            started_at=datetime.now(),
            completed_at=datetime.now(),
            timed_out=True,
        )
        assert result.succeeded is False

    def test_to_dict(self):
        """Test serialization to dict."""
        result = AsyncActionResult(
            action=AsyncAction(action_id="a1", action_name="test"),
            trigger_location=1.5,
            completion_location=2.0,
            started_at=datetime(2024, 1, 1, 12, 0, 0),
            completed_at=datetime(2024, 1, 1, 12, 0, 1),
        )
        d = result.to_dict()
        assert d["action_id"] == "a1"
        assert d["action_name"] == "test"
        assert d["trigger_location"] == 1.5
        assert d["completion_location"] == 2.0
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

        action = AsyncAction(action_id="a1", action_name="log_action")
        action_location = ActionLocation(path_parameter=1.0, action=action)

        executor = AsyncActionExecutor(
            motion_group_id="test",
            executor_actions=[action_location],
            execution_state=ExecutionState(),
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
            ActionLocation(
                path_parameter=3.0, action=AsyncAction(action_id="a3", action_name="log")
            ),
            ActionLocation(
                path_parameter=1.0, action=AsyncAction(action_id="a1", action_name="log")
            ),
            ActionLocation(
                path_parameter=2.0, action=AsyncAction(action_id="a2", action_name="log")
            ),
        ]

        executor = AsyncActionExecutor(
            motion_group_id="test",
            executor_actions=actions,
            execution_state=ExecutionState(),
            error_mode=ErrorHandlingMode.COLLECT,
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
    async def test_await_action_pauses_until_complete(self):
        """Test that AwaitAction pauses motion when the referenced action is still running."""
        execution_order = []
        pause_called = False
        resume_called = False

        async def slow_handler(ctx: ActionExecutionContext):
            execution_order.append("start")
            await asyncio.sleep(0.1)
            execution_order.append("end")

        register_async_action("slow", slow_handler)

        async def mock_pause():
            nonlocal pause_called
            pause_called = True

        async def mock_resume():
            nonlocal resume_called
            resume_called = True

        action = AsyncAction(action_id="s1", action_name="slow")
        await_act = AwaitAction(action_id="s1")
        actions = [
            ActionLocation(path_parameter=1.0, action=action),
            ActionLocation(path_parameter=2.0, action=await_act),
        ]

        executor = AsyncActionExecutor(
            motion_group_id="test",
            executor_actions=actions,
            execution_state=ExecutionState(),
            error_mode=ErrorHandlingMode.COLLECT,
            pause_callback=mock_pause,
            resume_callback=mock_resume,
        )

        state = self._make_robot_state()

        # Trigger the async action at location 1.0
        await executor.check_and_trigger(1.0, state)
        assert executor.has_running_actions

        # Trigger the await at location 2.0 — should pause, wait, resume
        await executor.check_and_trigger(2.0, state)

        assert pause_called
        assert resume_called
        assert execution_order == ["start", "end"]
        assert len(executor.results) == 1
        assert executor.results[0].succeeded

    @pytest.mark.asyncio
    async def test_await_action_already_completed(self):
        """Test that AwaitAction does not pause when action already finished."""
        pause_called = False

        async def fast_handler(ctx: ActionExecutionContext):
            pass

        register_async_action("fast", fast_handler)

        async def mock_pause():
            nonlocal pause_called
            pause_called = True

        action = AsyncAction(action_id="f1", action_name="fast")
        await_act = AwaitAction(action_id="f1")
        actions = [
            ActionLocation(path_parameter=1.0, action=action),
            ActionLocation(path_parameter=3.0, action=await_act),
        ]

        executor = AsyncActionExecutor(
            motion_group_id="test",
            executor_actions=actions,
            execution_state=ExecutionState(),
            error_mode=ErrorHandlingMode.COLLECT,
            pause_callback=mock_pause,
        )

        state = self._make_robot_state()
        await executor.check_and_trigger(1.0, state)
        await executor.wait_for_all_actions()
        # Action already completed by now

        await executor.check_and_trigger(3.0, state)
        assert not pause_called  # Should not pause

    @pytest.mark.asyncio
    async def test_await_with_timeout(self):
        """Test AwaitAction timeout handling."""

        async def forever_handler(ctx: ActionExecutionContext):
            await asyncio.sleep(100)

        register_async_action("forever", forever_handler)

        action = AsyncAction(action_id="f1", action_name="forever")
        await_act = AwaitAction(action_id="f1", timeout=0.1)
        actions = [
            ActionLocation(path_parameter=1.0, action=action),
            ActionLocation(path_parameter=2.0, action=await_act),
        ]

        executor = AsyncActionExecutor(
            motion_group_id="test",
            executor_actions=actions,
            execution_state=ExecutionState(),
            error_mode=ErrorHandlingMode.COLLECT,
            pause_callback=lambda: asyncio.sleep(0),
            resume_callback=lambda: asyncio.sleep(0),
        )

        state = self._make_robot_state()
        await executor.check_and_trigger(1.0, state)
        await executor.check_and_trigger(2.0, state)

        assert len(executor.results) == 1
        assert executor.results[0].timed_out is True
        assert executor.results[0].succeeded is False

    @pytest.mark.asyncio
    async def test_immediate_await_blocks_like_old_blocking(self):
        """Test start + immediate await at same location (old blocking=True equivalent)."""
        execution_order = []

        async def handler(ctx: ActionExecutionContext):
            execution_order.append("action")

        register_async_action("act", handler)

        action = AsyncAction(action_id="b1", action_name="act")
        await_act = AwaitAction(action_id="b1")
        actions = [
            ActionLocation(path_parameter=1.0, action=action),
            ActionLocation(path_parameter=1.0, action=await_act),
        ]

        executor = AsyncActionExecutor(
            motion_group_id="test",
            executor_actions=actions,
            execution_state=ExecutionState(),
            error_mode=ErrorHandlingMode.COLLECT,
            pause_callback=lambda: asyncio.sleep(0),
            resume_callback=lambda: asyncio.sleep(0),
        )

        state = self._make_robot_state()
        await executor.check_and_trigger(1.0, state)

        assert execution_order == ["action"]
        assert len(executor.results) == 1
        assert executor.results[0].succeeded

    @pytest.mark.asyncio
    async def test_dangling_await_raises_at_init(self):
        """Test that AwaitAction with no matching AsyncAction raises ValueError."""
        await_act = AwaitAction(action_id="nonexistent")
        actions = [ActionLocation(path_parameter=1.0, action=await_act)]

        with pytest.raises(ValueError, match="no corresponding AsyncAction"):
            AsyncActionExecutor(
                motion_group_id="test", executor_actions=actions, execution_state=ExecutionState()
            )

    @pytest.mark.asyncio
    async def test_error_mode_raise(self):
        """Test RAISE error mode propagates exceptions via await."""

        async def failing_handler(ctx: ActionExecutionContext):
            raise ValueError("test error")

        register_async_action("failing", failing_handler)

        action = AsyncAction(action_id="f1", action_name="failing")
        await_act = AwaitAction(action_id="f1")
        actions = [
            ActionLocation(path_parameter=1.0, action=action),
            ActionLocation(path_parameter=1.0, action=await_act),
        ]

        executor = AsyncActionExecutor(
            motion_group_id="test",
            executor_actions=actions,
            execution_state=ExecutionState(),
            error_mode=ErrorHandlingMode.RAISE,
            pause_callback=lambda: asyncio.sleep(0),
            resume_callback=lambda: asyncio.sleep(0),
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

        action = AsyncAction(action_id="f1", action_name="failing")
        await_act = AwaitAction(action_id="f1")
        actions = [
            ActionLocation(path_parameter=1.0, action=action),
            ActionLocation(path_parameter=1.0, action=await_act),
        ]

        executor = AsyncActionExecutor(
            motion_group_id="test",
            executor_actions=actions,
            execution_state=ExecutionState(),
            error_mode=ErrorHandlingMode.COLLECT,
            pause_callback=lambda: asyncio.sleep(0),
            resume_callback=lambda: asyncio.sleep(0),
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

        action = AsyncAction(action_id="f1", action_name="failing")
        await_act = AwaitAction(action_id="f1")
        actions = [
            ActionLocation(path_parameter=1.0, action=action),
            ActionLocation(path_parameter=1.0, action=await_act),
        ]

        executor = AsyncActionExecutor(
            motion_group_id="test",
            executor_actions=actions,
            execution_state=ExecutionState(),
            error_mode=ErrorHandlingMode.CALLBACK,
            error_callback=error_handler,
            pause_callback=lambda: asyncio.sleep(0),
            resume_callback=lambda: asyncio.sleep(0),
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
            ActionLocation(
                path_parameter=1.0, action=AsyncAction(action_id="a1", action_name="action")
            ),
            ActionLocation(
                path_parameter=2.0, action=AsyncAction(action_id="a2", action_name="action")
            ),
        ]

        executor = AsyncActionExecutor(
            motion_group_id="test-group", executor_actions=actions, execution_state=ExecutionState()
        )

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

        action = AsyncAction(action_id="l1", action_name="long")
        action_location = ActionLocation(path_parameter=1.0, action=action)

        executor = AsyncActionExecutor(
            motion_group_id="test",
            executor_actions=[action_location],
            execution_state=ExecutionState(),
        )

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
            items=(
                self._make_linear(),
                AsyncAction(action_id="a1", action_name="test"),
                self._make_linear(),
            )
        )
        assert len(actions) == 3

    def test_get_async_actions(self):
        """Test extracting async actions with locations."""
        actions = CombinedActions(
            items=(
                self._make_linear(),  # location 1
                AsyncAction(action_id="a1", action_name="first"),  # at location 1
                self._make_linear(),  # location 2
                self._make_linear(),  # location 3
                AsyncAction(action_id="a2", action_name="second"),  # at location 3
            )
        )

        async_actions = actions.get_async_actions()
        assert len(async_actions) == 2
        assert async_actions[0].path_parameter == 1.0
        assert async_actions[0].action.action_name == "first"
        assert async_actions[1].path_parameter == 3.0
        assert async_actions[1].action.action_name == "second"

    def test_get_executor_actions(self):
        """Test extracting executor actions including await and wait_until."""
        actions = CombinedActions(
            items=(
                self._make_linear(),
                AsyncAction(action_id="a1", action_name="test"),
                self._make_linear(),
                AwaitAction(action_id="a1"),
                WaitUntilAction(predicate=lambda s: True),
                self._make_linear(),
            )
        )

        executor_actions = actions.get_executor_actions()
        assert len(executor_actions) == 3
        assert isinstance(executor_actions[0].action, AsyncAction)
        assert isinstance(executor_actions[1].action, AwaitAction)
        assert isinstance(executor_actions[2].action, WaitUntilAction)

    def test_get_async_actions_without_motions_start_at_zero(self):
        """Test async-only action lists are mapped to location 0.0."""
        actions = CombinedActions(
            items=(
                AsyncAction(action_id="a1", action_name="first"),
                AsyncAction(action_id="a2", action_name="second"),
            )
        )

        async_actions = actions.get_async_actions()
        assert len(async_actions) == 2
        assert async_actions[0].path_parameter == 0.0
        assert async_actions[1].path_parameter == 0.0

    def test_get_executor_actions_without_motions_start_at_zero(self):
        """Test executor actions without motions are all mapped to location 0.0."""
        actions = CombinedActions(
            items=(
                AsyncAction(action_id="a1", action_name="test"),
                AwaitAction(action_id="a1"),
                WaitUntilAction(predicate=lambda s: True),
            )
        )

        executor_actions = actions.get_executor_actions()
        assert len(executor_actions) == 3
        assert [action.path_parameter for action in executor_actions] == [0.0, 0.0, 0.0]

    def test_get_executor_actions_before_first_motion_use_zero_location(self):
        """Test executor actions before the first motion use the start location."""
        actions = CombinedActions(
            items=(
                AsyncAction(action_id="a1", action_name="test"),
                AwaitAction(action_id="a1"),
                self._make_linear(),
            )
        )

        executor_actions = actions.get_executor_actions()
        assert len(executor_actions) == 2
        assert [action.path_parameter for action in executor_actions] == [0.0, 0.0]

    def test_motions_excludes_async_actions(self):
        """Test that motions property doesn't include async actions."""
        actions = CombinedActions(
            items=(
                self._make_linear(),
                AsyncAction(action_id="a1", action_name="test"),
                self._make_linear(),
            )
        )
        assert len(actions.motions) == 2

    def test_to_set_io_excludes_async_actions(self):
        """Test that to_set_io doesn't include async actions."""
        actions = CombinedActions(
            items=(self._make_linear(), AsyncAction(action_id="a1", action_name="test"))
        )
        # Should not raise and should return empty (no WriteActions)
        io_list = actions.to_set_io()
        assert io_list == []


# ============================================================================
# ExecutionState Tests
# ============================================================================


class TestExecutionState:
    """Tests for ExecutionState class."""

    @pytest.mark.asyncio
    async def test_set_and_get(self):
        """Test basic set/get operations."""
        state = ExecutionState()
        await state.set("key", "value")
        assert state.get("key") == "value"
        assert state.get("missing") is None
        assert state.get("missing", 42) == 42

    @pytest.mark.asyncio
    async def test_wait_for_already_true(self):
        """Test wait_for returns immediately when predicate is already True."""
        state = ExecutionState()
        await state.set("ready", True)
        result = await state.wait_for(lambda s: s.get("ready"), timeout=1.0)
        assert result is True

    @pytest.mark.asyncio
    async def test_wait_for_becomes_true(self):
        """Test wait_for blocks until predicate becomes True via concurrent set."""
        state = ExecutionState()

        async def set_after_delay():
            await asyncio.sleep(0.05)
            await state.set("done", True)

        asyncio.create_task(set_after_delay())
        result = await state.wait_for(lambda s: s.get("done"), timeout=2.0)
        assert result is True

    @pytest.mark.asyncio
    async def test_wait_for_timeout(self):
        """Test wait_for returns False on timeout."""
        state = ExecutionState()
        result = await state.wait_for(lambda s: s.get("never_set"), timeout=0.05)
        assert result is False

    @pytest.mark.asyncio
    async def test_snapshot(self):
        """Test snapshot returns a copy."""
        state = ExecutionState()
        await state.set("a", 1)
        snap = state.snapshot()
        assert snap == {"a": 1}
        snap["b"] = 2  # mutating snapshot should not affect state
        assert state.get("b") is None


# ============================================================================
# AwaitAction / WaitUntilAction Factory Tests
# ============================================================================


class TestAwaitActionFactory:
    """Tests for await_action factory function."""

    def test_basic_creation(self):
        action = await_action("a1")
        assert action.action_id == "a1"
        assert action.timeout is None
        assert action.is_motion() is False

    def test_with_timeout(self):
        action = await_action("a1", timeout=5.0)
        assert action.timeout == 5.0


class TestWaitUntilFactory:
    """Tests for wait_until factory function."""

    def test_basic_creation(self):
        def pred(s):
            return s.get("x")

        action = wait_until(pred)
        assert action.predicate is pred
        assert action.timeout is None
        assert action.is_motion() is False

    def test_with_timeout(self):
        action = wait_until(lambda s: True, timeout=3.0)
        assert action.timeout == 3.0


# ============================================================================
# WaitUntilAction Executor Integration Tests
# ============================================================================


class TestWaitUntilExecutor:
    """Tests for WaitUntilAction integration with AsyncActionExecutor."""

    def teardown_method(self):
        get_default_registry().clear()

    def _make_robot_state(self) -> RobotState:
        return RobotState(
            pose=Pose(position=(0, 0, 0), orientation=(0, 0, 0)),
            tcp="flange",
            joints=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        )

    @pytest.mark.asyncio
    async def test_predicate_already_true(self):
        """Test WaitUntil does not pause when predicate is already True."""
        pause_called = False

        async def mock_pause():
            nonlocal pause_called
            pause_called = True

        execution_state = ExecutionState()
        await execution_state.set("ready", True)

        wait_act = WaitUntilAction(predicate=lambda s: s.get("ready"), timeout=1.0)
        actions = [ActionLocation(path_parameter=1.0, action=wait_act)]

        executor = AsyncActionExecutor(
            motion_group_id="test",
            executor_actions=actions,
            execution_state=execution_state,
            pause_callback=mock_pause,
        )

        state = self._make_robot_state()
        await executor.check_and_trigger(1.0, state)
        assert not pause_called

    @pytest.mark.asyncio
    async def test_predicate_becomes_true_via_async_action(self):
        """Test WaitUntil pauses and resumes when async action sets state."""
        pause_called = False
        resume_called = False

        async def setter(ctx: ActionExecutionContext):
            await asyncio.sleep(0.05)
            await ctx.state.set("part_detected", True)

        register_async_action("detect", setter)

        async def mock_pause():
            nonlocal pause_called
            pause_called = True

        async def mock_resume():
            nonlocal resume_called
            resume_called = True

        execution_state = ExecutionState()
        action = AsyncAction(action_id="d1", action_name="detect")
        wait_act = WaitUntilAction(predicate=lambda s: s.get("part_detected"), timeout=2.0)
        actions = [
            ActionLocation(path_parameter=1.0, action=action),
            ActionLocation(path_parameter=2.0, action=wait_act),
        ]

        executor = AsyncActionExecutor(
            motion_group_id="test",
            executor_actions=actions,
            execution_state=execution_state,
            pause_callback=mock_pause,
            resume_callback=mock_resume,
        )

        state = self._make_robot_state()
        await executor.check_and_trigger(1.0, state)
        await executor.check_and_trigger(2.0, state)

        assert pause_called
        assert resume_called
        assert execution_state.get("part_detected") is True

    @pytest.mark.asyncio
    async def test_predicate_timeout(self):
        """Test WaitUntil with timeout when predicate never becomes True."""
        execution_state = ExecutionState()
        wait_act = WaitUntilAction(predicate=lambda s: s.get("never"), timeout=0.05)
        actions = [ActionLocation(path_parameter=1.0, action=wait_act)]

        executor = AsyncActionExecutor(
            motion_group_id="test",
            executor_actions=actions,
            execution_state=execution_state,
            pause_callback=lambda: asyncio.sleep(0),
            resume_callback=lambda: asyncio.sleep(0),
        )

        state = self._make_robot_state()
        paused = await executor.check_and_trigger(1.0, state)
        assert paused is True
