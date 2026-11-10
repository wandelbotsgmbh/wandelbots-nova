"""Measured-velocity parsing, staleness, and the client's fallback.

No broker and no robot: the parsing is a pure function and the client's velocity
choice is decided by what the source returns.
"""

from __future__ import annotations

import time

import numpy as np
import pytest

from novapolicy.nats.client import PROPRIO_KEY, NatsPolicyClient, _Block
from novapolicy.nats.protocol import ActionSpec
from novapolicy.state_stream import (
    DEFAULT_STALE_AFTER_S,
    StateStreamVelocities,
    VelocitySource,
    _parse_velocities,
)

DOF = 7
MG_ID = "1@manus"
JOINTS = np.arange(DOF, dtype=np.float32) / 10.0
MEASURED = tuple(float(v) for v in np.linspace(0.1, 0.7, DOF))


# -- parsing ----------------------------------------------------------------


def test_velocities_are_read_from_the_result_envelope() -> None:
    frame = '{"result": {"joint_position": [0,0,0,0,0,0,0], "joint_velocity": [1,2,3,4,5,6,7]}}'
    assert _parse_velocities(frame) == (1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0)


@pytest.mark.parametrize(
    "frame",
    [
        '{"result": {"joint_position": [0,0]}}',  # the field the typed model keeps
        '{"result": {"joint_velocity": []}}',
        '{"result": {"joint_velocity": null}}',
        '{"message": "Not Found"}',
        "not json at all",
        '{"result": {"joint_velocity": "fast"}}',
    ],
)
def test_a_frame_without_usable_velocities_is_skipped(frame: str) -> None:
    """A frame that carries none must not overwrite the last good sample."""
    assert _parse_velocities(frame) is None


def test_non_numeric_entries_reject_the_whole_frame() -> None:
    assert _parse_velocities('{"result": {"joint_velocity": [1, 2, "x"]}}') is None


# -- freshness --------------------------------------------------------------


def test_an_unseen_motion_group_has_no_velocities() -> None:
    source = StateStreamVelocities([])
    assert source.velocities(MG_ID) is None


def test_a_sample_expires() -> None:
    """Past the staleness bound the caller must fall back, not reuse an old value.

    A stale velocity is worse than a noisy one: it describes motion that has
    already finished.
    """
    source = StateStreamVelocities([], stale_after_s=DEFAULT_STALE_AFTER_S)
    # Backdate the sample rather than patch the clock: the staleness bound is
    # measured against arrival time, so an old arrival is the same test.
    source._latest[MG_ID] = (time.monotonic(), MEASURED)
    assert source.velocities(MG_ID) == MEASURED
    source._latest[MG_ID] = (
        time.monotonic() - DEFAULT_STALE_AFTER_S - 0.01,
        MEASURED,
    )
    assert source.velocities(MG_ID) is None


def test_sampled_reports_what_has_arrived() -> None:
    source = StateStreamVelocities([])
    assert source.sampled == {}
    source._latest[MG_ID] = (0.0, MEASURED)
    assert source.sampled == {MG_ID: DOF}


def test_it_satisfies_the_protocol() -> None:
    assert isinstance(StateStreamVelocities([]), VelocitySource)


# -- the client's choice ----------------------------------------------------


class _Source:
    """A velocity source under the test's control."""

    def __init__(self, values: tuple[float, ...] | None) -> None:
        self.values = values
        self.started = False
        self.stopped = False

    async def start(self) -> None:
        self.started = True

    async def stop(self) -> None:
        self.stopped = True

    def velocities(self, motion_group_id: str) -> tuple[float, ...] | None:
        del motion_group_id
        return self.values


def _client(source: _Source | None) -> tuple[NatsPolicyClient, _Block]:
    client = NatsPolicyClient(policy_id="p", servers="nats://x", velocities=source)
    spec = ActionSpec(name="joint_target", dof=DOF, horizon=1, dt_ms=20.0)
    block = _Block(PROPRIO_KEY, "joint_target", spec)
    block.motion_group_id = MG_ID
    return client, block


def test_measured_velocities_are_used_when_present() -> None:
    client, block = _client(_Source(MEASURED))
    got = client._velocities(block, JOINTS, now=1.0)
    np.testing.assert_allclose(got, MEASURED, rtol=1e-6)


def test_measured_velocities_win_over_a_finite_difference() -> None:
    """The finite difference is the fallback, not a blend or an average."""
    client, block = _client(_Source(MEASURED))
    client._prev_joints[MG_ID] = JOINTS - 1.0
    client._prev_t[MG_ID] = 0.0
    got = client._velocities(block, JOINTS, now=0.02)
    np.testing.assert_allclose(got, MEASURED, rtol=1e-6)


def test_without_a_source_it_finite_differences() -> None:
    client, block = _client(None)
    client._prev_joints[MG_ID] = JOINTS - 0.02
    client._prev_t[MG_ID] = 0.0
    got = client._velocities(block, JOINTS, now=0.02)
    np.testing.assert_allclose(got, np.ones(DOF, dtype=np.float32), rtol=1e-5)


def test_a_source_with_nothing_yet_falls_back() -> None:
    client, block = _client(_Source(None))
    client._prev_joints[MG_ID] = JOINTS - 0.02
    client._prev_t[MG_ID] = 0.0
    got = client._velocities(block, JOINTS, now=0.02)
    np.testing.assert_allclose(got, np.ones(DOF, dtype=np.float32), rtol=1e-5)


def test_a_width_mismatch_falls_back_rather_than_raising() -> None:
    """A misconfigured source must not stop a moving robot."""
    client, block = _client(_Source((0.1, 0.2)))
    client._prev_joints[MG_ID] = JOINTS
    client._prev_t[MG_ID] = 0.0
    got = client._velocities(block, JOINTS, now=0.02)
    assert got.shape == (DOF,)
    np.testing.assert_allclose(got, np.zeros(DOF, dtype=np.float32))


def test_the_first_step_reports_zero_without_a_source() -> None:
    client, block = _client(None)
    got = client._velocities(block, JOINTS, now=0.0)
    np.testing.assert_allclose(got, np.zeros(DOF, dtype=np.float32))
