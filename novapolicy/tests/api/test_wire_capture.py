"""What reaches NOVA, pinned per scenario: the guard for moving shaping between places.

Each scenario runs the real executor (or the jogger's calls) against real
``WaypointJoggingSession`` objects whose loops are not started and whose clock is frozen and
advanced step by step. After every queued chunk the session builds the request it would send,
and the test records the waypoints (time and values), ``delay_until_braking``, the step
timestamps and when the chunk's IO is due. The record must match ``data/wire_golden.json``
exactly.

Regenerate only on purpose, after checking the diff: ``UPDATE_WIRE_GOLDEN=1 pytest <this file>``.
"""

from __future__ import annotations

import asyncio
import json
import os
from pathlib import Path
from typing import TYPE_CHECKING, Any
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from novapolicy.executor import PolicyExecutor
from novapolicy.jogging.waypoint_session import WaypointJoggingSession
from novapolicy.joint_limits import JointLimits
from novapolicy.policy_client import CallbackPolicyClient
from novapolicy.queued_client import QueuedPolicyClient
from novapolicy.schema import Observation, PolicySchema
from novapolicy.types import (
    ActionChunk,
    ContinuousExecution,
    SequentialExecution,
    WaypointConfig,
)

if TYPE_CHECKING:
    from collections.abc import Awaitable, Callable

GOLDEN = Path(__file__).parent / "data" / "wire_golden.json"
MG_ID = "0@ur10e"
START_MS = 10_000
LIMITS = JointLimits(lower=(-2.0,) * 6, upper=(2.0,) * 6)


class _Clock:
    """A frozen stand-in for the session's server clock, moved only by the test."""

    def __init__(self, now_ms: float) -> None:
        self.now_ms = now_ms
        self.last_sample_wall = None
        self.state_clock_calibrated = True

    @property
    def estimated_server_timestamp_ms(self) -> float:
        return self.now_ms

    @property
    def acknowledged_elapsed_ms(self) -> float:
        return self.now_ms

    @property
    def last_server_timestamp_ms(self) -> int:
        return int(self.now_ms)

    def start(self) -> None:
        pass


def _round(values: list[float]) -> list[float]:
    return [round(float(v), 9) for v in values]


class _Capture:
    """Real sessions behind a fake transport; records what each one would send."""

    def __init__(self, *, advance_ms: float) -> None:
        self.records: list[dict[str, Any]] = []
        self.advance_ms = advance_ms
        self.sessions: dict[str, WaypointJoggingSession] = {}

    def make(self, *args: object, **kwargs: Any) -> MagicMock:  # ruff: ignore[too-many-statements]
        real = WaypointJoggingSession(*args, **kwargs)
        clock = _Clock(START_MS)
        real._clock = clock  # ty: ignore[invalid-assignment]
        real._current_joints = [0.0] * 6
        real._joint_limits = LIMITS  # what start() reads for a JointClamp
        mg_id = kwargs["motion_group"].id
        self.sessions[mg_id] = real
        proxy = MagicMock()
        proxy.motion_group_id = mg_id
        proxy.has_failed = False
        proxy.failure_reason = ""
        proxy.failure_exception = None
        proxy.stop_condition_triggered = None
        proxy.is_running = True
        proxy.jogging_state = None
        proxy.is_at_standstill = True
        proxy.standstill_ms = 1000.0
        proxy.queued_chunk_count = 0
        proxy.scheduled_chunk_count = 0
        proxy.scheduled_until_server_ms = 0
        proxy.start = AsyncMock()
        proxy.stop = AsyncMock()
        proxy.drain = AsyncMock(return_value=True)
        proxy.wait_ready = AsyncMock()
        proxy.write_ios = AsyncMock()
        step_timestamps: list[int] = []

        def sync_clock() -> None:
            proxy.session_elapsed_ms = int(clock.now_ms)
            proxy.last_server_timestamp_ms = int(clock.now_ms)
            proxy.estimated_server_timestamp_ms = clock.now_ms
            proxy.current_state_server_ms = int(clock.now_ms) - 5
            proxy.executing_action_timestep = real.executing_action_timestep
            state = MagicMock()
            state.joints = tuple(real._current_joints or [0.0] * 6)
            state.pose = state.tcp = None
            proxy.current_state = state

        def scheduled_timestamp_for_step(step: int) -> int | None:
            return step_timestamps[step] if 0 <= step < len(step_timestamps) else None

        def update_chunk(*a: object, **kw: Any) -> None:
            real.update_chunk(*a, **kw)
            proxy.queued_chunk_count = real.queued_chunk_count
            pending = real._pending_request
            assert pending is not None
            # What a real session reports until its loop sends the chunk, at the next await.
            proxy.queued_chunk = pending
            real._pending_request = None
            scheduled = real._trimmed_request(pending)
            request = scheduled.request
            sent = [] if request is None else list(request.waypoints)
            due = None
            if pending.when_due is not None and scheduled.step_timestamps:
                step = pending.when_due[0]
                due = [
                    step,
                    scheduled.step_timestamps[min(step, len(scheduled.step_timestamps) - 1)],
                ]
            self.records.append({
                "group": mg_id,
                "now_ms": clock.now_ms,
                "lead": pending.lead,
                "action_timestep": pending.action_timestep,
                "skipped": scheduled.skipped,
                "step_timestamps": list(scheduled.step_timestamps),
                "sent": [
                    [w.timestamp, _round(getattr(w.waypoint, "joints", None) or [])] for w in sent
                ],
                "delay_until_braking_ms": (
                    None if request is None else request.delay_until_braking
                ),
                "io_due": due,
            })
            step_timestamps[:] = scheduled.step_timestamps
            real._record_timesteps(
                pending, scheduled.step_timestamps, scheduled.skipped
            )  # as the loop does
            proxy.scheduled_chunk_count = pending.sequence
            if scheduled.step_timestamps:
                proxy.scheduled_until_server_ms = scheduled.step_timestamps[
                    pending.last_caller_step
                ]
            if sent:
                real._current_joints = list(getattr(sent[-1].waypoint, "joints", []) or [])
                real._last_sent_value = list(real._current_joints)
            clock.now_ms = max(
                clock.now_ms + self.advance_ms, float(proxy.scheduled_until_server_ms)
            )
            sync_clock()

        proxy.update_chunk = MagicMock(side_effect=update_chunk)
        proxy.scheduled_timestamp_for_step = MagicMock(side_effect=scheduled_timestamp_for_step)
        proxy.queued_chunk = None
        sync_clock()
        return proxy


def _mg() -> MagicMock:
    mg = MagicMock()
    mg.id = MG_ID
    mg._controller_id = "ur10e"
    mg._cell = "cell"
    mg._api_client = MagicMock(close=AsyncMock())
    return mg


def _schema() -> PolicySchema:
    return PolicySchema(observations=[Observation.joint_positions("arm", source=_mg())])


def _chunks(chunks: list[ActionChunk]) -> Callable[[object], Awaitable[ActionChunk]]:
    """A policy that returns ``chunks`` in turn and stops the run after the last."""
    calls = iter(range(len(chunks) + 1))
    executor_ref: list[PolicyExecutor] = []

    async def get_actions(_obs: object) -> ActionChunk:
        i = next(calls)
        if i >= len(chunks) - 1:
            executor_ref[0].stop()
        return chunks[min(i, len(chunks) - 1)]

    get_actions.executor_ref = executor_ref  # type: ignore[attr-defined]
    return get_actions


def _ramp(start: float, n: int, step: float) -> list[list[float]]:
    return [[start + step * k, 0.1, 0.2, 0.0, 0.0, 0.0] for k in range(n)]


async def _run_executor(
    capture: _Capture, policy_fn: Callable[[object], Awaitable[ActionChunk]], **kwargs: Any
) -> None:
    policy = kwargs.pop("policy", None) or CallbackPolicyClient(policy_fn)
    estop = MagicMock(start=AsyncMock(), stop=AsyncMock(), error=None)
    with (
        patch("novapolicy.executor.WaypointJoggingSession", side_effect=capture.make),
        patch("novapolicy.executor.EstopMonitor", return_value=estop),
        patch("novapolicy.rerun._is_rerun_active", return_value=False),
    ):
        executor = PolicyExecutor(_schema(), policy, timeout_s=5.0, **kwargs)
        getattr(policy_fn, "executor_ref", []).append(executor)
        await asyncio.wait_for(executor.run(), 10)


def _sequential_bridge_and_ramp() -> tuple[_Capture, Any]:
    capture = _Capture(advance_ms=0.0)
    fn = _chunks([
        ActionChunk(joints={MG_ID: _ramp(0.1, 5, 0.05)}, dt_ms=1000 / 30),
        ActionChunk(joints={MG_ID: _ramp(0.4, 5, 0.05)}, dt_ms=1000 / 30),
    ])
    return capture, _run_executor(capture, fn, execution=SequentialExecution())


def _continuous_one_step() -> tuple[_Capture, Any]:
    capture = _Capture(advance_ms=1000 / 30)
    fn = _chunks([ActionChunk(joints={MG_ID: _ramp(0.01 * k, 1, 0.0)}) for k in range(5)])
    return capture, _run_executor(
        capture,
        fn,
        execution=ContinuousExecution(rate_hz=30),
        motion=WaypointConfig(delay_until_braking_ms=200),
    )


def _continuous_multi_step() -> tuple[_Capture, Any]:
    capture = _Capture(advance_ms=100.0)
    fn = _chunks([
        ActionChunk(joints={MG_ID: _ramp(0.1 * k, 10, 0.02)}, dt_ms=1000 / 30) for k in range(3)
    ])
    return capture, _run_executor(capture, fn, execution=ContinuousExecution(rate_hz=10))


def _out_of_limits() -> tuple[_Capture, Any]:
    capture = _Capture(advance_ms=100.0)
    steps = [[1.5, 2.5, -3.0, 0.0, 0.0, 0.0], [1.9, 2.6, -3.1, 0.0, 0.0, 0.0]]
    fn = _chunks([ActionChunk(joints={MG_ID: steps}, dt_ms=50.0)] * 2)
    return capture, _run_executor(capture, fn, execution=ContinuousExecution(rate_hz=10))


def _io_at_first_step() -> tuple[_Capture, Any]:
    capture = _Capture(advance_ms=0.0)
    fn = _chunks([
        ActionChunk(
            joints={MG_ID: _ramp(0.1, 4, 0.05)}, dt_ms=50.0, ios={MG_ID: {"digital_out[0]": True}}
        ),
        ActionChunk(
            joints={MG_ID: _ramp(0.3, 4, 0.05)}, dt_ms=50.0, ios={MG_ID: {"digital_out[0]": False}}
        ),
    ])
    return capture, _run_executor(capture, fn, execution=SequentialExecution())


def _timestep_queue() -> tuple[_Capture, Any]:
    capture = _Capture(advance_ms=1000 / 30)
    chunks = [
        ActionChunk(joints={MG_ID: _ramp(0.05 * k, 8, 0.01)}, dt_ms=1000 / 30) for k in range(4)
    ]
    fn = _chunks(chunks)
    queued = QueuedPolicyClient(CallbackPolicyClient(fn))
    return capture, _run_executor(
        capture, fn, policy=queued, execution=ContinuousExecution(rate_hz=30)
    )


async def _jog(chunk: bool) -> list[dict[str, Any]]:
    capture = _Capture(advance_ms=11.0)
    proxy = capture.make(motion_group=_mg(), config=WaypointConfig(), mode="joint")
    session = capture.sessions[MG_ID]
    for k in range(4):
        now = session.estimated_server_timestamp_ms
        if chunk:
            proxy.update_chunk(
                steps=_ramp(0.02 * k, 6, 0.01),
                dt_ms=20.0,
                first_timestamp_ms=int(now + 100),
                extend_buffer=True,
            )
        else:
            proxy.update_chunk(
                steps=_ramp(0.01 * k, 1, 0.0),
                dt_ms=0.0,
                first_timestamp_ms=int(now + 100),
                timestamp_offset_steps=0,
                observed_at_ms=int(now),
                extend_buffer=False,
            )
    return capture.records


SCENARIOS: dict[str, Callable[[], tuple[_Capture, Any]]] = {
    "sequential_bridge_and_ramp": _sequential_bridge_and_ramp,
    "continuous_one_step": _continuous_one_step,
    "continuous_multi_step": _continuous_multi_step,
    "out_of_limits": _out_of_limits,
    "io_at_first_step": _io_at_first_step,
    "timestep_queue": _timestep_queue,
}


async def _capture_all() -> dict[str, list[dict[str, Any]]]:
    out: dict[str, list[dict[str, Any]]] = {}
    for name, build in SCENARIOS.items():
        capture, run = build()
        await run
        out[name] = capture.records
    out["jog_live_target"] = await _jog(chunk=False)
    out["jog_chunk"] = await _jog(chunk=True)
    return out


@pytest.mark.asyncio
async def test_what_reaches_nova_is_unchanged() -> None:
    captured = json.loads(json.dumps(await _capture_all()))
    if os.environ.get("UPDATE_WIRE_GOLDEN"):
        GOLDEN.parent.mkdir(exist_ok=True)
        GOLDEN.write_text(json.dumps(captured, indent=1) + "\n")
    golden = json.loads(GOLDEN.read_text())
    for name in golden:
        assert captured.get(name) == golden[name], name
    assert set(captured) == set(golden)
