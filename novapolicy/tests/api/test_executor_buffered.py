"""``PolicyExecutor``'s buffered mode: what reaches the session, and when.

The transport is faked, so these assert the executor's own decisions — that a
target is replayed as a horizon rather than sent alone, that the horizon lands on
one fixed timeline across sends, and that the mode refuses inputs it would have
to discard.
"""

from __future__ import annotations

from typing import cast
from unittest.mock import AsyncMock, MagicMock

import pytest

from novapolicy.executor import PolicyExecutor
from novapolicy.jogging.live_buffer import LIVE_LEAD_MS
from novapolicy.policy_client import CallbackPolicyClient
from novapolicy.schema import Observation, PolicySchema
from novapolicy.types import ActionChunk

MG_ID = "0@robot"
DOF = 6
WINDOW_MS = 500.0
SINGLE_STEP_DT_MS = 50.0
TICK_MS = 20.0
"""Server time between sends, i.e. the policy's own step."""


def _session() -> MagicMock:
    session = MagicMock()
    session.motion_group_id = MG_ID
    session.session_elapsed_ms = 0.0
    session.single_step_dt_ms = SINGLE_STEP_DT_MS
    session.queued_chunk_count = 0
    session.start = AsyncMock()
    session.stop = AsyncMock()
    session.wait_ready = AsyncMock()
    sent: list[dict] = []
    session.sent = sent

    def update_chunk(**kwargs: object) -> None:
        session.queued_chunk_count += 1
        sent.append(kwargs)

    session.update_chunk = MagicMock(side_effect=update_chunk)
    return session


def _executor(window_ms: float = WINDOW_MS) -> tuple[PolicyExecutor, MagicMock]:
    mg = MagicMock()
    mg.id = MG_ID
    schema = PolicySchema(observations=[Observation.joint_positions("joints", source=mg)])

    async def _never_called(_obs: object) -> ActionChunk:
        raise AssertionError("these tests drive _send directly")

    executor = PolicyExecutor(
        schema, CallbackPolicyClient(_never_called), buffer_window_ms=window_ms
    )
    session = _session()
    executor._sessions = {MG_ID: session}
    return executor, session


def _chunk(value: float) -> ActionChunk:
    return ActionChunk(joints={MG_ID: [[value] * DOF]}, dt_ms=20.0)


def _feed(executor: PolicyExecutor, count: int, *, step: float = 0.01) -> None:
    """Send ``count`` targets, advancing the server clock one policy step each.

    The buffer takes its spacing from how far apart its samples sit on the
    server timeline, so a feed that does not advance that clock measures a span
    of zero and asserts nothing about the real replay.
    """
    # The executor's session map is typed for the real transport; here it holds
    # the fake, whose clock the test drives.
    session = cast("MagicMock", next(iter(executor._sessions.values())))
    for i in range(count):
        session.session_elapsed_ms = 1000.0 + i * TICK_MS
        executor._send(_chunk(i * step))


# -- what reaches the session -----------------------------------------------


def _send_one(executor: PolicyExecutor, session: MagicMock, value: float = 0.0) -> None:
    session.session_elapsed_ms = 1000.0
    executor._send(_chunk(value))


def test_the_first_target_goes_out_alone() -> None:
    """One sample cannot define a spacing, so there is no horizon to replay yet.

    Sending it anyway keeps the robot live; waiting for the window to fill would
    mean never moving at all.
    """
    executor, session = _executor()
    _send_one(executor, session)
    assert len(session.sent) == 1
    assert len(session.sent[0]["steps"]) == 1
    assert session.sent[0]["dt_ms"] == 0.0
    assert session.sent[0]["extend_buffer"] is False


def test_a_filled_buffer_is_replayed_as_a_horizon() -> None:
    executor, session = _executor()
    _feed(executor, 40)
    last = session.sent[-1]
    assert len(last["steps"]) > 1, "a horizon, not a lone terminal waypoint"
    assert last["dt_ms"] == pytest.approx(SINGLE_STEP_DT_MS, rel=0.5)
    assert last["extend_buffer"] is False, "the replay is the horizon; no hold padding"


def test_the_padding_is_never_used_in_this_mode() -> None:
    """Holding one pose is not the same as replaying where the target went."""
    executor, session = _executor()
    _feed(executor, 40)
    assert all(sent["extend_buffer"] is False for sent in session.sent)


def test_nothing_replayed_is_past_the_newest_target() -> None:
    executor, session = _executor()
    _feed(executor, 40)
    newest = 39 * 0.01
    assert max(step[0] for step in session.sent[-1]["steps"]) <= newest + 1e-9


# -- the timeline -----------------------------------------------------------


def test_every_send_carries_an_absolute_first_timestamp() -> None:
    """Never a "now"-relative placement: that is the fault this mode avoids."""
    executor, session = _executor()
    _feed(executor, 40)
    horizons = [sent for sent in session.sent if len(sent["steps"]) > 1]
    assert horizons
    for sent in horizons:
        assert sent.get("first_timestamp_ms") is not None
        assert sent.get("timestamp_offset_steps", 0) == 0


def test_the_timeline_anchor_is_taken_once() -> None:
    """Re-anchoring later would move already-commanded points."""
    executor, session = _executor()
    _send_one(executor, session)
    first_anchor = executor._buffer_anchor_ms
    _feed(executor, 20)
    assert executor._buffer_anchor_ms == first_anchor


def test_the_grid_spacing_settles_once_the_window_is_full() -> None:
    """While filling, the spacing is half the span, so it changes every send.

    Only once the window covers more than twice the configured step does the
    spacing settle, and only then is a target's absolute time stable. Anything
    asserting timestamp stability has to start after that.
    """
    executor, session = _executor()
    _feed(executor, 60)
    settled = [sent["dt_ms"] for sent in session.sent[-20:] if len(sent["steps"]) > 1]
    assert settled
    assert all(dt == pytest.approx(SINGLE_STEP_DT_MS) for dt in settled)


def test_a_target_keeps_its_timestamp_as_the_window_is_resent() -> None:
    """The property the mode exists for, through the executor this time.

    Measured over the settled window only (see the test above): a target that
    appears in several consecutive replays must be commanded at one absolute
    time in all of them.
    """
    executor, session = _executor()
    _feed(executor, 80)
    settled = [
        sent
        for sent in session.sent
        if len(sent["steps"]) > 1 and sent["dt_ms"] == pytest.approx(SINGLE_STEP_DT_MS)
    ]
    assert len(settled) > 5
    stamps: dict[tuple[float, ...], set[int]] = {}
    for sent in settled:
        base = sent["first_timestamp_ms"]
        dt = sent["dt_ms"]
        for index, step in enumerate(sent["steps"]):
            stamps.setdefault(tuple(step), set()).add(base + int(index * dt))
    repeated = {k: v for k, v in stamps.items() if len(v) > 1}
    assert not repeated, f"{len(repeated)} targets were commanded at more than one time"


def test_the_anchor_leads_the_session_clock() -> None:
    executor, session = _executor()
    _feed(executor, 40)
    horizons = [sent for sent in session.sent if len(sent["steps"]) > 1]
    assert horizons[0]["first_timestamp_ms"] >= 1_000.0 + LIVE_LEAD_MS


# -- refusals ---------------------------------------------------------------


def test_a_multi_step_chunk_is_refused() -> None:
    """A chunk carries its own horizon; taking steps[0] would discard the rest."""
    executor, session = _executor()
    session.session_elapsed_ms = 1000.0
    chunk = ActionChunk(joints={MG_ID: [[0.0] * DOF, [0.1] * DOF]}, dt_ms=20.0)
    with pytest.raises(ValueError, match="horizon-1"):
        executor._send(chunk)


def test_tcp_targets_are_refused() -> None:
    executor, _ = _executor()
    chunk = ActionChunk(joints={}, tcp={MG_ID: [[0.0] * 6]}, dt_ms=20.0)
    with pytest.raises(ValueError, match="joint targets only"):
        executor._send(chunk)


def test_zero_window_keeps_the_chunk_path() -> None:
    """The default must not change behaviour for anyone."""
    executor, session = _executor(window_ms=0.0)
    assert executor._buffer_window_ms == 0.0
    _send_one(executor, session)
    assert session.sent, "the chunk path still sends"
    # The chunk path leaves extend_buffer at its default, so the hold padding
    # still applies there; only the buffered mode opts out.
    assert "extend_buffer" not in session.sent[0]
