"""Regression: move_forward must neither hang nor falsely abort when the
controller's terminal completion event is lost / delayed on the state stream.

Frames are captured from a live ur5e (``fixtures/frames_swallow.json``) and
recombined to model each scenario. Every test drives the REAL move_forward
controller against a fake websocket that stays open (so the error-consumer
racer never rescues us — exactly like the field hang).
"""

from __future__ import annotations

import asyncio
import json
from pathlib import Path

import pytest

import nova.api as api
from nova.actions import CombinedActions, MovementControllerContext
from nova.cell.movement_controller.move_forward import move_forward
from nova.exceptions import MovementStalled

FIXTURE = Path(__file__).parent / "fixtures" / "frames_swallow.json"


def _load() -> tuple[list[api.models.MotionGroupState], list[float]]:
    data = json.loads(FIXTURE.read_text())
    frames = [api.models.MotionGroupState.model_validate(f) for f in data["frames"]]
    return frames, list(data["target_joints"])


def _is_ended(s: api.models.MotionGroupState) -> bool:
    return (
        s.execute is not None
        and isinstance(s.execute.details, api.models.TrajectoryDetails)
        and isinstance(s.execute.details.state, api.models.TrajectoryEnded)
    )


def _is_running(s: api.models.MotionGroupState) -> bool:
    return (
        s.execute is not None
        and isinstance(s.execute.details, api.models.TrajectoryDetails)
        and isinstance(s.execute.details.state, api.models.TrajectoryRunning)
    )


def _with(s, *, standstill=None, joints=None, execute_none=False):
    f = s.model_copy(deep=True)
    if standstill is not None:
        f.standstill = standstill
    if joints is not None:
        f.joint_position = api.models.DoubleArray(root=list(joints))
    if execute_none:
        f.execute = None
    return f


class _FakeResponseStream:
    """execute_trajectory websocket: driver pushes the Init/Start acks; then
    ``__anext__`` blocks forever -> the ws stays OPEN, as in the field hang."""

    def __init__(self) -> None:
        self._q: asyncio.Queue = asyncio.Queue()

    def __aiter__(self):
        return self

    async def __anext__(self):
        return await self._q.get()

    def push(self, item) -> None:
        self._q.put_nowait(item)


async def _drive(state_gen_factory, target, **ctx_kw) -> str:
    ctx = MovementControllerContext(
        combined_actions=CombinedActions(items=tuple()),
        motion_id="test-trajectory",
        start_on_io=None,
        pause_on_io=None,
        motion_group_state_stream_gen=state_gen_factory,
        target_joint_position=tuple(target) if target is not None else None,
        stall_timeout_s=ctx_kw.pop("stall_timeout_s", 0.4),
        **ctx_kw,
    )
    resp = _FakeResponseStream()
    gen = move_forward(ctx)(resp)
    async for req in gen:
        root = getattr(req, "root", req)
        if isinstance(root, api.models.InitializeMovementRequest):
            resp.push(api.models.ExecuteTrajectoryResponse(api.models.InitializeMovementResponse()))
        elif isinstance(root, api.models.StartMovementRequest):
            resp.push(api.models.ExecuteTrajectoryResponse(api.models.StartMovementResponse()))
    return "returned"


def _replay_then_rest(frames, rest):
    async def gen():
        for f in frames:
            yield f
        while True:  # robot at rest, stream alive, terminal event never comes
            yield rest
            await asyncio.sleep(0.005)

    return gen


# --------------------------------------------------------------------------- #
# Existing behaviour (kept)
# --------------------------------------------------------------------------- #


@pytest.mark.asyncio
async def test_swallowed_terminal_window_does_not_hang():
    frames, target = _load()
    kept = [f for f in frames if not (_is_ended(f) and f.standstill)]  # swallow the window
    rest = _with(kept[-1], standstill=True, joints=target, execute_none=True)
    await asyncio.wait_for(_drive(_replay_then_rest(kept, rest), target), timeout=5.0)


@pytest.mark.asyncio
async def test_healthy_completion_still_works():
    frames, target = _load()
    rest = _with(frames[-1], standstill=True, joints=target, execute_none=True)
    await asyncio.wait_for(_drive(_replay_then_rest(frames, rest), target), timeout=5.0)


@pytest.mark.asyncio
async def test_swallow_in_ending_completes_via_standstill_not_watchdog():
    """Change A: once TrajectoryEnded is seen (machine in `ending`), a bare
    standstill frame (rae drops `execute` at teardown) completes IMMEDIATELY and
    tolerance-free. With the watchdog timeout set very high, only A can resolve
    this within the window -> proves A, not B, does it."""
    frames, target = _load()
    kept = [f for f in frames if not (_is_ended(f) and f.standstill)]  # swallow window
    rest = _with(kept[-1], standstill=True, joints=target, execute_none=True)
    await asyncio.wait_for(
        _drive(_replay_then_rest(kept, rest), target, stall_timeout_s=30.0, max_stall_s=60.0),
        timeout=2.0,
    )


@pytest.mark.asyncio
async def test_ending_standstill_completes_even_off_target():
    """Change A trusts the controller's explicit TrajectoryEnded: a standstill in
    `ending` completes without an at-target check, so a slightly-off reading (e.g.
    a linear rail vs a tight tolerance) does not turn a finished move into a false
    error. The genuine stop-short case (no TrajectoryEnded) is caught by B's hard
    ceiling via the `executing` path instead."""
    frames, target = _load()
    kept = [f for f in frames if not (_is_ended(f) and f.standstill)]
    off_target = [t + 0.5 for t in target]
    rest = _with(kept[-1], standstill=True, joints=off_target, execute_none=True)
    await asyncio.wait_for(
        _drive(_replay_then_rest(kept, rest), target, stall_timeout_s=30.0, max_stall_s=60.0),
        timeout=2.0,
    )


# --------------------------------------------------------------------------- #
# New correctness scenarios
# --------------------------------------------------------------------------- #


@pytest.mark.asyncio
async def test_stuck_in_executing_at_target_completes():
    """TrajectoryEnded never delivered (machine stuck in 'executing') but the
    robot reached target -> watchdog confirms at-target and completes."""
    frames, target = _load()
    running = [f for f in frames if _is_running(f)]
    # only running frames (no TrajectoryEnded ever), ending at target standstill
    seq = running[:30]
    rest = _with(running[-1], standstill=True, joints=target, execute_none=True)
    await asyncio.wait_for(_drive(_replay_then_rest(seq, rest), target), timeout=5.0)


@pytest.mark.asyncio
async def test_zero_motion_at_target_completes():
    """A move where the robot never leaves standstill (already at target) must
    still complete, not hang, even with the terminal event lost."""
    frames, target = _load()
    running = [f for f in frames if _is_running(f)]
    at_rest = _with(running[0], standstill=True, joints=target)  # never moves
    rest = _with(running[0], standstill=True, joints=target, execute_none=True)
    await asyncio.wait_for(_drive(_replay_then_rest([at_rest, at_rest], rest), target), timeout=5.0)


@pytest.mark.asyncio
async def test_legitimate_mid_path_dwell_is_not_aborted():
    """A standstill mid-trajectory (machine still 'executing', not at target)
    must NOT be aborted; the move completes when motion resumes."""
    frames, target = _load()
    running = [f for f in frames if _is_running(f)]
    ended_idx = next(i for i, f in enumerate(frames) if _is_ended(f))
    ended_seq = frames[ended_idx:]  # TrajectoryEnded ss=0 ... ss=1 @ target
    mid_joints = list(running[len(running) // 2].joint_position.root)  # not the target
    dwell = _with(running[len(running) // 2], standstill=True, joints=mid_joints)
    rest = _with(frames[-1], standstill=True, joints=target, execute_none=True)

    async def gen():
        loop = asyncio.get_running_loop()
        for f in running[:5]:  # moving
            yield f
        t0 = loop.time()  # hold a dwell longer than stall_timeout_s
        while loop.time() - t0 < 0.7:
            yield dwell
            await asyncio.sleep(0.02)
        for f in running[5:10]:  # resume
            yield f
        for f in ended_seq:  # real completion
            yield f
        while True:
            yield rest
            await asyncio.sleep(0.005)

    # stall_timeout_s=0.3 < 0.7s dwell: the old watchdog would abort here.
    await asyncio.wait_for(_drive(gen, target, stall_timeout_s=0.3), timeout=5.0)


@pytest.mark.asyncio
async def test_stuck_in_executing_not_at_target_hits_hard_ceiling():
    """A genuine stop-short with no TrajectoryEnded (machine still 'executing',
    robot not at target) must not hang forever: the hard ceiling surfaces it."""
    frames, target = _load()
    running = [f for f in frames if _is_running(f)]
    seq = running[:10]  # moves a little, never ends
    off_target = [t + 0.5 for t in target]
    rest = _with(running[-1], standstill=True, joints=off_target, execute_none=True)
    with pytest.raises(MovementStalled):
        await asyncio.wait_for(
            _drive(_replay_then_rest(seq, rest), target, stall_timeout_s=0.3, max_stall_s=0.6),
            timeout=5.0,
        )


@pytest.mark.asyncio
async def test_at_target_tolerance_is_configurable():
    """A small residual offset within the configured tolerance still counts as
    at-target (covers loose/linear-axis settling)."""
    frames, target = _load()
    kept = [f for f in frames if not (_is_ended(f) and f.standstill)]
    near = [t + 0.05 for t in target]  # 0.05 off; inside a 0.1 tolerance
    rest = _with(kept[-1], standstill=True, joints=near, execute_none=True)
    await asyncio.wait_for(
        _drive(_replay_then_rest(kept, rest), target, at_target_tolerance=0.1), timeout=5.0
    )


def _with_state(s, state_obj):
    """Clone a running frame and swap its trajectory-state discriminator."""
    f = s.model_copy(deep=True)
    f.execute.details.state = state_obj
    return f


@pytest.mark.asyncio
async def test_io_wait_standstill_is_not_treated_as_stall():
    """A robot legitimately at standstill while WAITING_FOR_IO must NOT be
    completed or aborted by the watchdog (Roberto's pause/IO path). Even past the
    soft and hard timeouts, an IO wait is excluded; the move completes only when
    IO clears and the trajectory actually ends."""
    frames, target = _load()
    running = [f for f in frames if _is_running(f)]
    ended_idx = next(i for i, f in enumerate(frames) if _is_ended(f))
    ended_seq = frames[ended_idx:]
    waiting = _with_state(
        _with(running[len(running) // 2], standstill=True), api.models.TrajectoryWaitForIO()
    )
    rest = _with(frames[-1], standstill=True, joints=target, execute_none=True)

    async def gen():
        loop = asyncio.get_running_loop()
        for f in running[:5]:
            yield f
        t0 = loop.time()  # hold WAITING_FOR_IO at standstill past BOTH timeouts
        while loop.time() - t0 < 0.8:
            yield waiting
            await asyncio.sleep(0.02)
        for f in running[5:10]:  # IO cleared, motion resumes
            yield f
        for f in ended_seq:
            yield f
        while True:
            yield rest
            await asyncio.sleep(0.005)

    # max_stall_s=0.5 < 0.8s wait: if WAITING_FOR_IO were not excluded, the hard
    # ceiling would wrongly raise. It must complete instead.
    await asyncio.wait_for(_drive(gen, target, stall_timeout_s=0.3, max_stall_s=0.5), timeout=5.0)


@pytest.mark.asyncio
async def test_machine_completion_and_watchdog_do_not_double_resolve():
    """When the machine completes on the bare standstill (Change A), the armed
    watchdog must be cancelled cleanly — no spurious MovementStalled, no double
    resolution — even with a tiny stall timeout that races completion."""
    frames, target = _load()
    kept = [f for f in frames if not (_is_ended(f) and f.standstill)]
    rest = _with(kept[-1], standstill=True, joints=target, execute_none=True)
    # Tiny timeouts so the watchdog is eligible to fire almost immediately; the
    # machine's standstill-after-end completion must still win and return cleanly.
    await asyncio.wait_for(
        _drive(_replay_then_rest(kept, rest), target, stall_timeout_s=0.02, max_stall_s=0.05),
        timeout=5.0,
    )
