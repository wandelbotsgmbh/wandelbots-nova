"""Tests for the shared IO-overlay helpers in ``nova.actions.container``.

``located_writes`` anchors each write on the count of preceding motions (waits
skipped); ``to_set_io`` builds on it and must keep its historical behavior."""

from nova import api
from nova.actions.container import CombinedActions, located_writes, write_to_set_io
from nova.actions.io import io_write
from nova.actions.mock import wait
from nova.actions.motions import joint_ptp


def _ptp(sample: float):
    return joint_ptp((sample,) * 6)


class TestLocatedWrites:
    def test_index_counts_preceding_motions(self):
        writes = located_writes(
            [
                io_write("OUT#0", True),  # 0 motions before
                _ptp(1.0),
                io_write("OUT#1", True),  # 1 motion before
                _ptp(2.0),
                _ptp(3.0),
                io_write("OUT#3", True),  # 3 motions before
            ]
        )
        assert [(index, w.key) for index, w in writes] == [(0, "OUT#0"), (1, "OUT#1"), (3, "OUT#3")]

    def test_wait_does_not_advance_index(self):
        writes = located_writes([_ptp(1.0), wait(0.5), io_write("OUT#1", True)])
        assert [index for index, _ in writes] == [1]

    def test_no_writes__empty(self):
        assert located_writes([_ptp(1.0), wait(0.5)]) == []

    def test_write_to_set_io_carries_value_location_and_origin(self):
        write = io_write("OUT#1", True, origin=api.models.IOOrigin.BUS_IO)
        set_io = write_to_set_io(write, 2)
        assert set_io.location == 2
        assert set_io.io_origin is api.models.IOOrigin.BUS_IO
        assert set_io.io.root.io == "OUT#1"


class TestToSetIoRegression:
    def test_matches_located_writes_over_mixed_list(self):
        items = [_ptp(1.0), io_write("OUT#1", True), wait(0.5), _ptp(2.0), io_write("OUT#2", False)]
        combined = CombinedActions(items=tuple(items))

        result = combined.to_set_io()

        assert [s.location for s in result] == [1, 2]
        assert [s.io.root.io for s in result] == ["OUT#1", "OUT#2"]
        assert [s.io.root.value for s in result] == [True, False]
