"""The send pipeline end to end on a virtual robot: jogging, a dummy VLA's chunks continuously
and sequentially, a timestep queue, and one-step targets.

One throwaway virtual UR10e for the module (deleted on teardown). Each test records, per motion
group, every waypoint the session sends and every state the controller reports (server ms), and
checks that the arm followed what was commanded: the measured joint against the latest commanded
value for the same moment. Needs NOVA_API (and NOVA_ACCESS_TOKEN where required); skipped unless
run with ``-m integration``.
"""

from __future__ import annotations

import asyncio
import contextlib
import math
import time
from typing import TYPE_CHECKING, Any
import uuid

import numpy as np
import pytest
import pytest_asyncio

from novapolicy import (
    ContinuousExecution,
    LiveBuffer,
    PolicyExecutor,
    QueuedPolicyClient,
    SequentialExecution,
    default_pipeline,
    jog_joints,
)
from novapolicy.jogging import waypoint_session as ws
from novapolicy.policy_client import CallbackPolicyClient
from novapolicy.schema import Action, Observation, PolicySchema
from novapolicy.types import ActionChunk

if TYPE_CHECKING:
    from collections.abc import Iterator

START = [0.0, -1.2, 1.4, -1.8, -1.57, 0.0]
AMPLITUDE_RAD, FREQUENCY_HZ = 0.25, 0.4
DT_MS = 1000.0 / 30.0
LAG_MS = 8.0  # the controller follows its waypoints this much later


def _target(t_s: float) -> list[float]:
    """The trajectory every scenario follows: joint 0 swings, the rest hold."""
    return [START[0] + AMPLITUDE_RAD * math.sin(2 * math.pi * FREQUENCY_HZ * t_s), *START[1:]]


@pytest_asyncio.fixture(scope="module", loop_scope="module")
async def ur10e():
    from nova import Nova, api
    from nova.cell import virtual_controller

    name = f"pipeline-itest-{uuid.uuid4().hex[:8]}"
    async with Nova() as nova_instance, contextlib.AsyncExitStack() as stack:
        cell = nova_instance.cell()
        controller = await cell.ensure_controller(
            virtual_controller(
                name=name,
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type="universalrobots-ur10e",
            )
        )

        async def _cleanup() -> None:
            with contextlib.suppress(Exception):
                await cell.delete_robot_controller(name, timeout=25)

        stack.push_async_callback(_cleanup)
        yield await stack.enter_async_context(controller[0])


class _Recording:
    """Every waypoint sent and every state reported, on the server's clock."""

    def __init__(self) -> None:
        self.sent: list[tuple[int, float]] = []
        self.states: list[tuple[int, float]] = []
        self.requests = 0
        self.spans: list[tuple[int, int]] = []


@pytest.fixture
def recording(monkeypatch: pytest.MonkeyPatch) -> Iterator[_Recording]:
    record = _Recording()
    trimmed, notify = (
        ws.WaypointJoggingSession._trimmed_request,
        ws.WaypointJoggingSession._notify_state_observer,
    )

    def recording_trimmed(self: Any, pending: Any) -> Any:
        scheduled = trimmed(self, pending)
        if scheduled.request is not None:
            record.requests += 1
            waypoints = scheduled.request.waypoints
            record.spans.append((waypoints[0].timestamp, waypoints[-1].timestamp))
            record.sent.extend(
                (w.timestamp, w.waypoint.joints[0]) for w in scheduled.request.waypoints
            )
        return scheduled

    def recording_notify(self: Any, ts_ms: int) -> None:
        if self._current_joints is not None:
            record.states.append((ts_ms, self._current_joints[0]))
        notify(self, ts_ms)

    monkeypatch.setattr(ws.WaypointJoggingSession, "_trimmed_request", recording_trimmed)
    monkeypatch.setattr(ws.WaypointJoggingSession, "_notify_state_observer", recording_notify)
    yield record


def _tracking_deg(record: _Recording, *, skip_ms: float = 500.0) -> tuple[float, float]:
    """RMS and max of the measured joint 0 against the latest value commanded for the same
    moment (the latest request wins a timestamp), at the moments some request covered, after
    ``skip_ms``: between sequential chunks nothing is commanded."""
    latest = dict(record.sent)  # the latest request wins a timestamp
    times = np.array(sorted(latest), dtype=float)
    commanded = np.array([latest[int(t)] for t in times])
    states = np.array(sorted(set(record.states)), dtype=float)
    at = states[:, 0] - LAG_MS
    covered = np.zeros(len(at), dtype=bool)
    for first, last in record.spans:
        covered |= (at >= first) & (at <= last)
    window = covered & (at >= times[0] + skip_ms)
    assert window.sum() > 50, "too few states inside the commanded span"
    error = np.abs(np.interp(states[window, 0] - LAG_MS, times, commanded) - states[window, 1])
    return math.degrees(float(np.sqrt(np.mean(error**2)))), math.degrees(float(error.max()))


def _schema(mg: Any) -> PolicySchema:
    return PolicySchema(
        observations=[Observation.joint_positions("arm", source=mg, action=False)],
        actions=[Action.joint_positions("arm", target=mg, mode="absolute")],
    )


def _dummy_vla(mg: Any, steps: int) -> CallbackPolicyClient:
    """A VLA stand-in: each call plans ``steps`` steps of the trajectory from now, after 30 ms
    of "inference"."""
    t0: list[float] = []

    async def policy(_obs: object) -> ActionChunk:
        if not t0:
            t0.append(time.monotonic())
        now = time.monotonic() - t0[0]
        await asyncio.sleep(0.03)
        plan = [_target(now + k * DT_MS / 1000) for k in range(steps)]
        return ActionChunk(joints={mg.id: plan}, dt_ms=DT_MS)

    return CallbackPolicyClient(policy)


async def _run(mg: Any, policy: Any, execution: Any, duration_s: float = 6.0) -> str:
    result = await PolicyExecutor(
        _schema(mg),
        policy,
        execution=execution,
        n_action_steps=0,
        start_joint_position={mg: START},
        timeout_s=duration_s,
    ).run()
    return result.reason


@pytest.mark.integration
@pytest.mark.asyncio(loop_scope="module")
async def test_jogging_follows_live_targets_through_a_live_buffer(ur10e, recording):
    async with jog_joints(
        ur10e, start_joint_position=START, pipeline=[LiveBuffer(500), *default_pipeline()]
    ) as jogger:
        async for _ in jogger:
            if jogger.elapsed >= 5.0:
                break
            jogger.set_target(_target(jogger.elapsed))
    rms, peak = _tracking_deg(recording)
    assert rms < 0.1 and peak < 1.0, (rms, peak)


@pytest.mark.integration
@pytest.mark.asyncio(loop_scope="module")
async def test_a_dummy_vla_runs_continuously(ur10e, recording):
    reason = await _run(ur10e, _dummy_vla(ur10e, 15), ContinuousExecution(rate_hz=10))
    assert reason == "timeout"
    rms, peak = _tracking_deg(recording)
    assert rms < 0.5 and peak < 2.0, (rms, peak)


@pytest.mark.integration
@pytest.mark.asyncio(loop_scope="module")
async def test_a_dummy_vla_runs_sequentially(ur10e, recording):
    """Chunk after chunk, each to standstill. The dummy plans by wall-clock time, so every chunk
    starts mid-swing from rest: the arm lags while it accelerates, then settles on the target."""
    reason = await _run(ur10e, _dummy_vla(ur10e, 15), SequentialExecution())
    assert reason == "timeout"
    assert recording.requests >= 5
    rms, peak = _tracking_deg(recording, skip_ms=0.0)
    assert rms < 1.0 and peak < 5.0, (rms, peak)
    last_commanded = max(recording.sent)[1]
    last_measured = max(recording.states)[1]
    assert (
        math.degrees(abs(last_measured - last_commanded)) < 0.2
    )  # the last state is read as the session stops


@pytest.mark.integration
@pytest.mark.asyncio(loop_scope="module")
async def test_a_timestep_queue_keeps_one_timeline(ur10e, recording):
    reason = await _run(
        ur10e, QueuedPolicyClient(_dummy_vla(ur10e, 15)), ContinuousExecution(rate_hz=10)
    )
    assert reason == "timeout"
    rms, peak = _tracking_deg(recording)
    assert rms < 0.5 and peak < 2.0, (rms, peak)


@pytest.mark.integration
@pytest.mark.asyncio(loop_scope="module")
async def test_one_step_targets_run_continuously(ur10e, recording):
    reason = await _run(ur10e, _dummy_vla(ur10e, 1), ContinuousExecution(rate_hz=30))
    assert reason == "timeout"
    rms, peak = _tracking_deg(recording)
    assert rms < 0.5 and peak < 2.0, (rms, peak)
