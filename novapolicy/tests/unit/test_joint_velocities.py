"""Joint velocities: measured where the controller sends them, finite-differenced otherwise."""

from __future__ import annotations

import time
from types import SimpleNamespace

import pytest

from novapolicy.mapping.joint_velocities import JointVelocities
from novapolicy.mapping.ops import Rad2Deg
from novapolicy.mapping.schema import Observation, PolicySchema
from novapolicy.tests.doubles import mg as _mg

MG_ID = "0@ur10e"


class _FakeMeasured:
    """A velocity source that returns whatever the test sets."""

    def __init__(self, values: tuple[float, ...] | None) -> None:
        self.values = values
        self.started = False

    async def start(self) -> None:
        self.started = True

    async def stop(self) -> None:
        self.started = False

    def velocities(self, motion_group_id: str) -> tuple[float, ...] | None:
        return self.values if motion_group_id == MG_ID else None


@pytest.mark.asyncio
async def test_measured_velocities_are_used_when_the_controller_sends_them():
    measured = _FakeMeasured((0.1, 0.2, 0.3))
    velocities = JointVelocities([_mg()], measured=measured)
    await velocities.start()
    assert velocities.read(MG_ID, [1.0, 2.0, 3.0], now=10.0) == [0.1, 0.2, 0.3]
    await velocities.stop()
    assert not measured.started


@pytest.mark.asyncio
async def test_without_a_measurement_positions_are_finite_differenced(caplog):
    velocities = JointVelocities([_mg()], measured=_FakeMeasured(None))
    await velocities.start()
    t0 = time.monotonic() + 1.0  # past the grace period for the first frame
    assert velocities.read(MG_ID, [0.0, 1.0], now=t0) == [0.0, 0.0]  # no history yet
    assert velocities.read(MG_ID, [0.1, 0.8], now=t0 + 0.5) == pytest.approx([0.2, -0.4])
    assert sum("finite-differencing" in r.message for r in caplog.records) == 1


@pytest.mark.asyncio
async def test_a_repeated_sample_keeps_the_velocity_instead_of_reading_standstill():
    velocities = JointVelocities([_mg()])
    velocities.read(MG_ID, [0.0], now=0.0)
    assert velocities.read(MG_ID, [0.1], now=0.1) == pytest.approx([1.0])
    assert velocities.read(MG_ID, [0.1], now=0.12) == pytest.approx([1.0])  # the same sample again
    assert velocities.read(MG_ID, [0.3], now=0.2) == pytest.approx([2.0])  # against the last change


@pytest.mark.asyncio
async def test_positions_unchanged_for_longer_than_a_repeat_read_as_standstill():
    velocities = JointVelocities([_mg()])
    velocities.read(MG_ID, [0.0], now=0.0)
    assert velocities.read(MG_ID, [0.1], now=0.1) == pytest.approx([1.0])
    assert velocities.read(MG_ID, [0.1], now=0.5) == [0.0]  # the arm stopped
    assert velocities.read(MG_ID, [0.2], now=0.6) == pytest.approx([1.0])  # from the stop


@pytest.mark.asyncio
async def test_a_missing_measurement_is_not_reported_before_the_first_frame_can_arrive(caplog):
    velocities = JointVelocities([_mg()], measured=_FakeMeasured(None))
    await velocities.start()
    velocities.read(MG_ID, [0.0])  # right after start
    assert not any("finite-differencing" in r.message for r in caplog.records)


@pytest.mark.asyncio
async def test_a_measurement_of_the_wrong_width_falls_back():
    velocities = JointVelocities([_mg()], measured=_FakeMeasured((0.5,)))
    await velocities.start()
    velocities.read(MG_ID, [0.0, 0.0], now=0.0)
    assert velocities.read(MG_ID, [0.1, 0.2], now=0.1) == pytest.approx([1.0, 2.0])


@pytest.mark.asyncio
async def test_the_schema_observes_joint_velocities_as_numbered_features():
    mg = _mg()
    schema = PolicySchema(
        observations=[
            Observation.joint_positions("q", source=mg, action=False),
            Observation.joint_velocities("qd", source=mg, ops=[Rad2Deg()]),
        ]
    )
    assert schema.get_motion_groups() == [mg]
    schema._joint_velocities = JointVelocities([mg], measured=_FakeMeasured((0.5, -0.5)))
    await schema.start_sources()
    obs = await schema.build_observation({MG_ID: SimpleNamespace(joints=(1.0, 2.0), pose=None)})
    assert obs["q_1"] == 1.0
    assert obs["qd_1"] == pytest.approx(28.6479, rel=1e-4)
    assert obs["qd_2"] == pytest.approx(-28.6479, rel=1e-4)
    await schema.stop_sources()


@pytest.mark.asyncio
async def test_sources_can_be_stopped_without_being_started():
    schema = PolicySchema(observations=[Observation.joint_velocities("qd", source=_mg())])
    await schema.stop_sources()
    obs = await schema.build_observation({MG_ID: SimpleNamespace(joints=(1.0,), pose=None)})
    assert obs == {"qd_1": 0.0}
