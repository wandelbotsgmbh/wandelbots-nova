"""Measured-velocity parsing, staleness, and the client's fallback.

No broker and no robot: the parsing is a pure function and the client's velocity
choice is decided by what the source returns.
"""

from __future__ import annotations

import time

import numpy as np
import pytest

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
