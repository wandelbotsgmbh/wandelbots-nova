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
from nova.actions.mock import wait
from nova.actions.motions import multi_collision_free
from nova.cell.multi_trajectory_cursor import IOSyncConfig, IOSyncDriver, SyncDriver
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


def _move():
    """A synchronized motion — the overlay counts it, its targets are irrelevant here."""
    return multi_collision_free({"a": (0.0,) * 6, "b": (0.0,) * 6})


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
                IOSyncConfig(
                    clear=io_write("sync-io", False),
                    release=io_write("sync-io", True),
                    watch={"a": watch_condition("sync-io")},
                ),
                IOGateway(),
                "cell",
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
        assert conditions["a"].io.root.io == "sync-io"
        assert conditions["a"].io.root.value is False
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
        assert _driver_of(executor).start_conditions()["a"].io.root.io == "sync-io"

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
    def _driver(self, config: IOSyncConfig, gateway=None) -> IOSyncDriver:
        return IOSyncDriver(config, gateway or IOGateway(), "cell")

    async def test_start_conditions__return_the_configured_conditions(self):
        condition = watch_condition("in-b", value=False)
        driver = self._driver(
            IOSyncConfig(
                clear=io_write("out-a", False, device_id="controller-a"),
                release=io_write("out-a", True, device_id="controller-a"),
                watch={"a": watch_condition("out-a"), "b": condition},
            )
        )

        assert driver.start_conditions()["b"] is condition
        assert driver.start_conditions()["a"].io.root.io == "out-a"

    async def test_clear_and_release__write_their_configured_values(self):
        gateway = IOGateway()
        driver = self._driver(
            IOSyncConfig(
                clear=io_write("sync-io", True, device_id="controller-a"),
                release=io_write("sync-io", False, device_id="controller-a"),
                watch={"a": watch_condition("sync-io", value=False)},
            ),
            gateway,
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
                [api.models.IOValue(api.models.IOBooleanValue(io="bus-sync", value=False))],
                [api.models.IOValue(api.models.IOBooleanValue(io="bus-sync", value=True))],
            ]
        )
        bus_trigger = io_write("bus-sync", True, origin=api.models.IOOrigin.BUS_IO)
        driver = self._driver(
            IOSyncConfig(
                clear=io_write("bus-sync", False, origin=api.models.IOOrigin.BUS_IO),
                release=bus_trigger,
                watch={"a": watch_condition("bus-sync")},
            ),
            gateway,
        )

        await driver.release()

        gateway.bus_ios_api.set_bus_io_values.assert_awaited_once()
        assert gateway.bus_ios_api.get_bus_io_values.await_count == 2


class TestIOOverlay:
    def _executor(
        self, gateway, controllers=("controller-a", "controller-b")
    ) -> TrajectoryExecutor:
        motion_groups = {
            "a": motion_group(gateway, controller=controllers[0]),
            "b": motion_group(gateway, controller=controllers[1]),
        }
        return TrajectoryExecutor(motion_groups, sync=sync_driver(gateway, groups=("a", "b")))

    def test_no_actions__all_overlays_empty(self):
        overlay = self._executor(IOGateway())._build_io_overlay(None)
        assert overlay == {"a": [], "b": []}

    def test_write_anchored_on_preceding_motion_count(self):
        overlay = self._executor(IOGateway())._build_io_overlay(
            [_move(), _move(), io_write("OUT#1", True, device_id="controller-a")]
        )
        assert [s.location for s in overlay["a"]] == [2]
        assert overlay["b"] == []

    def test_wait_does_not_advance_the_location(self):
        overlay = self._executor(IOGateway())._build_io_overlay(
            [_move(), wait(0.5), io_write("OUT#1", True, device_id="controller-a")]
        )
        assert [s.location for s in overlay["a"]] == [1]

    def test_controller_origin_routes_to_the_group_on_that_controller(self):
        overlay = self._executor(IOGateway())._build_io_overlay(
            [io_write("OUT#1", True, device_id="controller-b")]
        )
        assert overlay["a"] == []
        assert [s.io.root.io for s in overlay["b"]] == ["OUT#1"]

    def test_controller_origin_without_device_id__ambiguous__raises(self):
        with pytest.raises(ValueError, match="ambiguous"):
            self._executor(IOGateway())._build_io_overlay([io_write("OUT#1", True)])

    def test_controller_origin_without_device_id__shared_controller__routes_to_one(self):
        executor = self._executor(IOGateway(), controllers=("controller-a", "controller-a"))
        overlay = executor._build_io_overlay([io_write("OUT#1", True)])
        assert [s.io.root.io for s in overlay["a"]] == ["OUT#1"]
        assert overlay["b"] == []

    def test_controller_origin__no_matching_group__raises(self):
        with pytest.raises(ValueError, match="No motion group is on controller"):
            self._executor(IOGateway())._build_io_overlay(
                [io_write("OUT#1", True, device_id="controller-x")]
            )

    def test_bus_origin__fires_once_on_one_group(self):
        overlay = self._executor(IOGateway())._build_io_overlay(
            [io_write("bus-var", True, origin=api.models.IOOrigin.BUS_IO)]
        )
        # exactly one group carries it — a shared instant, not duplicated
        assert sum(len(entries) for entries in overlay.values()) == 1
        assert [s.io.root.io for s in overlay["a"]] == ["bus-var"]
        assert overlay["b"] == []


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
