"""Tests for how multi-motion-group execution is wired up.

Everything here is decided before or beside the ``executeTrajectory`` protocol:
what the builder assembles, what the executor rejects, what the sync driver
writes and what the drift monitor makes of two state streams. The session tests,
which need a fake gateway to speak that protocol, live in
``test_trajectory_executor_session.py``.
"""

from unittest.mock import AsyncMock, MagicMock

import pytest

from nova import api
from nova.actions.io import io_write
from nova.cell.multi_trajectory_cursor import IOSyncDriver, SyncDriver
from nova.cell.session_monitor import SyncDriftError, SyncDriftMonitor
from nova.cell.trajectory_executor import TrajectoryExecutor
from tests.cell.multi_group_doubles import (
    IOGateway,
    motion_group,
    multi_trajectory,
    running_state,
    sync_driver,
    watch_condition,
)

pytestmark = pytest.mark.asyncio


def _driver_of(executor: TrajectoryExecutor) -> SyncDriver:
    """The driver the builder assembled — the one place these tests reach inside."""
    return executor._sync


class TestExecutorValidation:
    async def test_driver_missing_a_group_condition__raises(self):
        gateway = IOGateway()

        with pytest.raises(ValueError, match="start condition"):
            TrajectoryExecutor(
                motion_groups={"a": motion_group(gateway), "b": motion_group(gateway)},
                sync=sync_driver(gateway, groups=("a",)),
            )

    async def test_controller_write_without_device_id__raises(self):
        with pytest.raises(ValueError, match="device_id"):
            IOSyncDriver(
                clear=io_write("sync-io", False),
                release=io_write("sync-io", True),
                watch={"a": watch_condition("sync-io")},
                api_client=IOGateway(),
                cell="cell",
            )

    async def test_trajectory_keys_not_matching_groups__raises(self):
        gateway = IOGateway()
        executor = TrajectoryExecutor(
            motion_groups={"a": motion_group(gateway)}, sync=sync_driver(gateway, groups=("a",))
        )

        with pytest.raises(ValueError, match="must match"):
            await executor.execute(multi_trajectory("other"))


class TestBuilder:
    def _two_groups(self, gateway: IOGateway) -> dict[str, MagicMock]:
        return {"a": motion_group(gateway), "b": motion_group(gateway)}

    async def test_build__derives_clear_and_watch_from_the_trigger(self):
        gateway = IOGateway()

        executor = (
            TrajectoryExecutor.builder(self._two_groups(gateway))
            .sync_on_io("sync-io", controller="controller-a", release_value=False)
            .build()
        )

        conditions = _driver_of(executor).start_conditions()
        assert conditions["a"].io.io == "sync-io"
        assert conditions["a"].io.value is False
        assert conditions["a"] == conditions["b"]
        await _driver_of(executor).clear()
        await _driver_of(executor).release()
        assert gateway.io_writes == [("sync-io", True), ("sync-io", False)]

    async def test_builder__plain_motion_groups__are_keyed_by_their_id(self):
        gateway = IOGateway()
        robot = motion_group(gateway)
        robot.id = "0@robot"
        positioner = motion_group(gateway)
        positioner.id = "0@positioner"

        executor = (
            TrajectoryExecutor.builder([robot, positioner])
            .sync_on_io("sync-io", controller="controller-a")
            .build()
        )

        assert set(executor._motion_groups) == {"0@robot", "0@positioner"}

    async def test_build__watch_override_replaces_the_derived_condition(self):
        gateway = IOGateway()
        wired_input = watch_condition("wired-in", value=False)

        executor = (
            TrajectoryExecutor.builder(self._two_groups(gateway))
            .sync_on_io("sync-io", controller="controller-a")
            .watch("b", wired_input)
            .build()
        )

        assert _driver_of(executor).start_conditions()["b"] is wired_input
        assert _driver_of(executor).start_conditions()["a"].io.io == "sync-io"

    async def test_build__writes_set_one_at_a_time__replace_what_sync_on_io_states(self):
        gateway = IOGateway()

        executor = (
            TrajectoryExecutor.builder(self._two_groups(gateway))
            .sync_on_io("sync-io", controller="controller-a")
            .release_io("release-io", True, controller="controller-a")
            .clear_io("clear-io", True, controller="controller-a")
            .build()
        )

        await _driver_of(executor).clear()
        await _driver_of(executor).release()
        assert gateway.io_writes == [("clear-io", True), ("release-io", True)]

    async def test_build__without_a_release_write__raises(self):
        builder = TrajectoryExecutor.builder(self._two_groups(IOGateway()))

        with pytest.raises(ValueError, match="release write"):
            builder.clear_io("clear-io", False, controller="controller-a").build()

    async def test_build__without_a_clear_write__raises(self):
        builder = TrajectoryExecutor.builder(self._two_groups(IOGateway()))

        with pytest.raises(ValueError, match="clear write"):
            builder.release_io("release-io", True, controller="controller-a").build()

    async def test_build__motion_groups_spanning_cells__raises(self):
        gateway = IOGateway()
        motion_groups = {
            "a": motion_group(gateway, cell="cell-a"),
            "b": motion_group(gateway, cell="cell-b"),
        }

        with pytest.raises(ValueError, match="one cell"):
            TrajectoryExecutor.builder(motion_groups).sync_on_io("sync-io").build()

    async def test_sync_on_io__ambiguous_controller__raises_on_the_call(self):
        gateway = IOGateway()
        motion_groups = {
            "a": motion_group(gateway, controller="controller-a"),
            "b": motion_group(gateway, controller="controller-b"),
        }
        builder = TrajectoryExecutor.builder(motion_groups)

        with pytest.raises(ValueError, match="ambiguous"):
            builder.sync_on_io("sync-io")

    async def test_watch__unknown_group__raises_on_the_call(self):
        builder = TrajectoryExecutor.builder(self._two_groups(IOGateway()))

        with pytest.raises(ValueError, match="Unknown motion group"):
            builder.watch("typo", watch_condition("in-a"))

    async def test_builder__without_motion_groups__raises(self):
        with pytest.raises(ValueError, match="At least one motion group"):
            TrajectoryExecutor.builder([])


class TestIOSyncDriver:
    def _driver(self, *, clear, release, watch, gateway=None) -> IOSyncDriver:
        return IOSyncDriver(
            clear=clear,
            release=release,
            watch=watch,
            api_client=gateway or IOGateway(),
            cell="cell",
        )

    async def test_start_conditions__return_the_configured_conditions(self):
        condition = watch_condition("in-b", value=False)
        driver = self._driver(
            clear=io_write("out-a", False, device_id="controller-a"),
            release=io_write("out-a", True, device_id="controller-a"),
            watch={"a": watch_condition("out-a"), "b": condition},
        )

        assert driver.start_conditions()["b"] is condition
        assert driver.start_conditions()["a"].io.io == "out-a"

    async def test_clear_and_release__write_their_configured_values(self):
        gateway = IOGateway()
        driver = self._driver(
            clear=io_write("sync-io", True, device_id="controller-a"),
            release=io_write("sync-io", False, device_id="controller-a"),
            watch={"a": watch_condition("sync-io", value=False)},
            gateway=gateway,
        )

        await driver.clear()
        await driver.release()

        assert gateway.trigger_writes == [True, False]

    async def test_release__bus_origin__writes_and_polls_until_observed(self):
        gateway = IOGateway()
        gateway.bus_ios_api = MagicMock()
        gateway.bus_ios_api.set_bus_io_values = AsyncMock()
        gateway.bus_ios_api.get_bus_io_values = AsyncMock(
            side_effect=[
                [api.models.IOBooleanValue(io="bus-sync", value=False)],
                [api.models.IOBooleanValue(io="bus-sync", value=True)],
            ]
        )
        bus_trigger = io_write("bus-sync", True, origin=api.models.IOOrigin.BUS_IO)
        driver = self._driver(
            clear=io_write("bus-sync", False, origin=api.models.IOOrigin.BUS_IO),
            release=bus_trigger,
            watch={"a": watch_condition("bus-sync")},
            gateway=gateway,
        )

        await driver.release()

        gateway.bus_ios_api.set_bus_io_values.assert_awaited_once()
        assert gateway.bus_ios_api.get_bus_io_values.await_count == 2


class TestSyncDriftMonitor:
    async def test_run__spread_beyond_threshold_at_same_timestamp__raises(self):
        async def stream_a():
            yield running_state(0.5, at_milliseconds=10)

        async def stream_b():
            yield running_state(0.9, at_milliseconds=10)

        monitor = SyncDriftMonitor(max_drift=0.2)

        with pytest.raises(SyncDriftError):
            await monitor.run({"a": stream_a(), "b": stream_b()})

    async def test_run__spread_within_threshold__completes(self):
        async def stream_a():
            yield running_state(0.5, at_milliseconds=10)

        async def stream_b():
            yield running_state(0.6, at_milliseconds=10)

        monitor = SyncDriftMonitor(max_drift=0.2)

        await monitor.run({"a": stream_a(), "b": stream_b()})
