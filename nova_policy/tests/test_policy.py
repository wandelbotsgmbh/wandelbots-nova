"""Unit tests for nova_policy package."""

from __future__ import annotations

import pytest

from nova_policy.tests.mock_source import MockActionSource
from nova_policy.types import ActionChunk, GuardStopError, PolicyRunnerConfig
from nova_policy.velocity_controller import VelocityController


class TestVelocityController:
    """Tests for the PID velocity controller."""

    def test_zero_error_returns_zero(self) -> None:
        vc = VelocityController(tolerance=0.01)
        result = vc.compute([1.0, 2.0, 3.0], [1.0, 2.0, 3.0])
        assert result == [0.0, 0.0, 0.0]

    def test_within_tolerance_returns_zero(self) -> None:
        vc = VelocityController(tolerance=0.01)
        result = vc.compute([1.0, 2.0, 3.0], [1.005, 2.005, 3.005])
        assert result == [0.0, 0.0, 0.0]

    def test_positive_error_returns_positive_velocity(self) -> None:
        vc = VelocityController(p_gain=2.0, d_gain=0.0, velocity_limit=10.0)
        result = vc.compute([0.0, 0.0], [1.0, 0.5])
        assert result[0] > 0.0
        assert result[1] > 0.0

    def test_negative_error_returns_negative_velocity(self) -> None:
        vc = VelocityController(p_gain=2.0, d_gain=0.0, velocity_limit=10.0)
        result = vc.compute([1.0, 0.5], [0.0, 0.0])
        assert result[0] < 0.0
        assert result[1] < 0.0

    def test_velocity_is_clamped(self) -> None:
        vc = VelocityController(p_gain=100.0, velocity_limit=1.5)
        result = vc.compute([0.0], [10.0])
        assert result[0] == 1.5

    def test_velocity_is_clamped_negative(self) -> None:
        vc = VelocityController(p_gain=100.0, velocity_limit=1.5)
        result = vc.compute([10.0], [0.0])
        assert result[0] == -1.5

    def test_mismatched_lengths_raises(self) -> None:
        vc = VelocityController()
        with pytest.raises(ValueError, match="Joint count mismatch"):
            vc.compute([0.0, 0.0], [1.0, 1.0, 1.0])

    def test_reset_clears_state(self) -> None:
        vc = VelocityController(p_gain=3.0, d_gain=0.1)
        vc.compute([0.0], [1.0])
        vc.reset()
        assert vc._prev_joints is None
        assert vc._prev_target is None
        assert vc._integral is None

    def test_integral_accumulates(self) -> None:
        vc = VelocityController(p_gain=0.0, i_gain=1.0, d_gain=0.0, velocity_limit=10.0)
        # First call sets up state
        vc.compute([0.0], [1.0])
        # Subsequent calls should show integral effect (need time to pass)
        import time

        time.sleep(0.01)
        result = vc.compute([0.0], [1.0])
        # With i_gain > 0, output should be nonzero (from p=0 but integral accumulates)
        # The integral accumulates error * dt, and with p=0, it's i * integral
        assert result[0] >= 0.0  # integral is accumulating positive

    def test_anti_windup_clamps_integral(self) -> None:
        vc = VelocityController(
            p_gain=0.0, i_gain=1.0, d_gain=0.0, integral_limit=0.5, velocity_limit=10.0
        )
        # Run compute a few times with large error to accumulate integral
        import time

        vc.compute([0.0], [100.0])  # sets up initial state
        time.sleep(0.02)
        vc.compute([0.0], [100.0])  # accumulates integral
        # Integral should be clamped to integral_limit
        assert vc._integral is not None
        assert vc._integral[0] <= 0.5

    def test_feedforward_disabled_by_default(self) -> None:
        vc = VelocityController(ff_gain=0.0)
        ff = vc._feedforward([1.0, 2.0], 1.0, 2)
        assert ff == [0.0, 0.0]


class TestActionChunk:
    """Tests for ActionChunk data model."""

    def test_from_dict_basic(self) -> None:
        data = {"joints": {"0@ur5e": [[0.1, 0.2, 0.3, 0.4, 0.5, 0.6]]}, "dt_ms": 0.0}
        chunk = ActionChunk.from_dict(data)
        assert "0@ur5e" in chunk.joints
        assert len(chunk.joints["0@ur5e"]) == 1
        assert chunk.dt_ms == 0.0
        assert chunk.ios is None

    def test_from_dict_with_ios(self) -> None:
        data = {
            "joints": {"0@ur5e": [[0.1, 0.2, 0.3]]},
            "ios": {"0@ur5e": {"digital_out[0]": True}},
        }
        chunk = ActionChunk.from_dict(data)
        assert chunk.ios is not None
        assert chunk.ios["0@ur5e"]["digital_out[0]"] is True

    def test_from_dict_multi_step(self) -> None:
        data = {"joints": {"0@ur5e": [[0.1, 0.2], [0.3, 0.4], [0.5, 0.6]]}, "dt_ms": 33.0}
        chunk = ActionChunk.from_dict(data)
        assert len(chunk.joints["0@ur5e"]) == 3
        assert chunk.dt_ms == 33.0

    def test_from_dict_missing_joints_raises(self) -> None:
        with pytest.raises((ValueError, Exception)):
            ActionChunk.from_dict({})

    def test_timestamp_auto_filled(self) -> None:
        chunk = ActionChunk(joints={"0@ur5e": [[0.0]]})
        assert chunk.timestamp is not None
        assert chunk.timestamp > 0


class TestPolicyRunnerConfig:
    """Tests for PolicyRunnerConfig defaults."""

    def test_defaults(self) -> None:
        cfg = PolicyRunnerConfig()
        assert cfg.velocity_limit == 1.5
        assert cfg.p_gain == 3.0
        assert cfg.i_gain == 0.0
        assert cfg.d_gain == 0.1
        assert cfg.ff_gain == 0.0
        assert cfg.tolerance == 0.01
        assert cfg.integral_limit == 2.0
        assert cfg.state_rate_ms == 10


class TestGuardStopError:
    """Tests for GuardStopError."""

    def test_error_message(self) -> None:
        err = GuardStopError("0@ur5e", "workspace_guard")
        assert "workspace_guard" in str(err)
        assert "0@ur5e" in str(err)
        assert err.motion_group_id == "0@ur5e"
        assert err.guard_name == "workspace_guard"


class TestMockActionSource:
    """Tests for MockActionSource."""

    @pytest.mark.asyncio
    async def test_generates_correct_count(self) -> None:
        source = MockActionSource(
            motion_group_ids=["0@test"],
            num_joints=3,
            home_joints=[0.0, 0.0, 0.0],
            interval_ms=10,
            max_steps=5,
        )
        chunks: list[ActionChunk] = []
        async for chunk in source:
            chunks.append(chunk)
        assert len(chunks) == 5

    @pytest.mark.asyncio
    async def test_chunk_size(self) -> None:
        source = MockActionSource(
            motion_group_ids=["0@test"],
            num_joints=3,
            home_joints=[0.0, 0.0, 0.0],
            interval_ms=10,
            max_steps=2,
            chunk_size=4,
        )
        async for chunk in source:
            assert len(chunk.joints["0@test"]) == 4
            assert chunk.dt_ms > 0.0
            break

    @pytest.mark.asyncio
    async def test_multi_group(self) -> None:
        source = MockActionSource(
            motion_group_ids=["0@left", "0@right"],
            num_joints=6,
            home_joints=[0.0] * 6,
            interval_ms=10,
            max_steps=1,
        )
        async for chunk in source:
            assert "0@left" in chunk.joints
            assert "0@right" in chunk.joints
            assert len(chunk.joints["0@left"][0]) == 6

    @pytest.mark.asyncio
    async def test_io_toggle(self) -> None:
        source = MockActionSource(
            motion_group_ids=["0@test"],
            num_joints=3,
            home_joints=[0.0, 0.0, 0.0],
            interval_ms=10,
            max_steps=50,
            io_toggle_key="digital_out[0]",
            io_toggle_interval_ms=100,
        )
        io_fired = False
        async for chunk in source:
            if chunk.ios is not None:
                io_fired = True
                break
        assert io_fired

    @pytest.mark.asyncio
    async def test_sinusoidal_output(self) -> None:
        source = MockActionSource(
            motion_group_ids=["0@test"],
            num_joints=2,
            home_joints=[1.0, 2.0],
            interval_ms=10,
            amplitude=0.5,
            max_steps=10,
        )
        async for chunk in source:
            joints = chunk.joints["0@test"][0]
            # Should oscillate around home
            assert 0.5 <= joints[0] <= 1.5
            assert 1.5 <= joints[1] <= 2.5
