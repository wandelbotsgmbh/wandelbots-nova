"""``PolicyExecutor`` with a ``VirtualPlant``: the policy sees the plant, the
ring buffer replays the plant's path, and a non-plant is refused up front."""

from __future__ import annotations

import asyncio
from unittest.mock import AsyncMock, MagicMock

import pytest

from nova.types import Pose, RobotState
from novapolicy.executor import PolicyExecutor
from novapolicy.joint_limits import JointLimits
from novapolicy.policy_client import CallbackPolicyClient
from novapolicy.schema import Observation, PolicySchema
from novapolicy.types import ActionChunk, ContinuousExecution

MG_ID = "0@robot"
DOF = 3
WINDOW_MS = 200.0


class FirstOrderPlant:
    """Every step moves the plant half way to the target: a servo with lag."""

    def __init__(self) -> None:
        self.state = [0.0] * DOF
        self.reset_from: list[float] | None = None

    def reset(self, states: dict[str, RobotState]) -> None:
        self.reset_from = list(states[MG_ID].joints)
        self.state = list(self.reset_from)

    def observe(self, states: dict[str, RobotState]) -> dict[str, RobotState]:
        return {
            group_id: state.model_copy(update={"joints": tuple(self.state)})
            for group_id, state in states.items()
        }

    def shape(self, chunk: ActionChunk) -> ActionChunk:
        target = chunk.joints[MG_ID][0]
        self.state = [q + 0.5 * (t - q) for q, t in zip(self.state, target, strict=True)]
        return chunk.model_copy(update={"joints": {MG_ID: [list(self.state)]}, "dt_ms": 20.0})


def _session() -> MagicMock:
    session = MagicMock()
    session.motion_group_id = MG_ID
    session.queued_chunk_count = 0
    session.session_elapsed_ms = 0.0
    session.single_step_dt_ms = 50.0
    session.start = AsyncMock()
    session.stop = AsyncMock()
    session.wait_ready = AsyncMock()
    session.sent = []

    def update_chunk(**kwargs: object) -> None:
        session.queued_chunk_count += 1
        session.sent.append(kwargs)

    session.update_chunk = MagicMock(side_effect=update_chunk)
    return session


def _executor(plant: object, window_ms: float = WINDOW_MS) -> tuple[PolicyExecutor, MagicMock]:
    mg = MagicMock()
    mg.id = MG_ID
    schema = PolicySchema(observations=[Observation.joint_positions("j", source=mg)])

    async def _never_called(_obs: object) -> ActionChunk:
        raise AssertionError("these tests drive _send directly")

    executor = PolicyExecutor(
        schema,
        CallbackPolicyClient(_never_called),
        execution=ContinuousExecution(),
        plant=plant,
        buffer_window_ms=window_ms,
    )
    session = _session()
    executor._sessions = {MG_ID: session}
    return executor, session


def _state(*joints: float) -> RobotState:
    return RobotState(pose=Pose((0, 0, 0, 0, 0, 0)), tcp=None, joints=joints)


def _feed(executor: PolicyExecutor, session: MagicMock, count: int) -> None:
    for k in range(count):
        session.session_elapsed_ms = k * 20.0
        executor._send(ActionChunk(joints={MG_ID: [[1.0, 1.0, 1.0]]}))


def test_rejects_objects_that_are_not_plants() -> None:
    with pytest.raises(TypeError, match="VirtualPlant"):
        _executor(plant=object())


def test_requires_the_ring_buffer() -> None:
    with pytest.raises(ValueError, match="buffer_window_ms"):
        _executor(FirstOrderPlant(), window_ms=0.0)


def test_plant_states_replace_robot_states_for_the_policy() -> None:
    plant = FirstOrderPlant()
    plant.state = [0.25, 0.25, 0.25]
    robot = {MG_ID: _state(1.0, 2.0, 3.0)}

    observed = plant.observe(robot)

    assert observed[MG_ID].joints == (0.25, 0.25, 0.25)
    assert observed[MG_ID].pose == robot[MG_ID].pose


def test_buffer_replays_the_plant_path_not_the_targets() -> None:
    plant = FirstOrderPlant()
    executor, session = _executor(plant)
    plant.reset({MG_ID: _state(0.0, 0.0, 0.0)})

    _feed(executor, session, 3)

    # One sample per send, the plant's state after that step: the buffer holds
    # the servo's path, never the raw 1.0 setpoint.
    samples = [values for _, values in executor._target_buffers[MG_ID]]
    assert [s[0] for s in samples] == pytest.approx([0.5, 0.75, 0.875])
    assert plant.state == pytest.approx([0.875] * DOF)
    assert len(session.sent[0]["steps"]) == 1  # bare target until the window fills
    assert all(max(step) <= 0.875 + 1e-9 for step in session.sent[-1]["steps"])


def test_clamp_applies_to_the_plant_path() -> None:
    plant = FirstOrderPlant()
    executor, session = _executor(plant)
    plant.reset({MG_ID: _state(0.0, 0.0, 0.0)})
    executor._joint_limits = {MG_ID: JointLimits(lower=(-1.0, -1.0, -1.0), upper=(0.6, 0.6, 0.6))}

    _feed(executor, session, 4)

    assert max(max(step) for sent in session.sent for step in sent["steps"]) == pytest.approx(0.6)


def test_plant_observation_is_logged_beside_the_robot_not_over_it() -> None:
    """With a plant, the policy's observation goes to the plant entities and
    target tracking is measured against the robot's live state."""
    plant = FirstOrderPlant()
    plant.state = [0.25, 0.25, 0.25]
    executor, session = _executor(plant)
    session.current_state = _state(1.0, 2.0, 3.0)
    rerun = MagicMock()
    executor._rerun = rerun

    observed = plant.observe({MG_ID: session.current_state})
    executor._log_policy_observation(observed, None, step=1)
    rerun.log_plant_observation.assert_called_once_with(observed, 1)
    rerun.log_observation.assert_not_called()

    asyncio.run(
        executor._send_policy_action(
            ActionChunk(joints={MG_ID: [[0.5, 0.5, 0.5]]}),
            observed,
            step=1,
            start_time=0.0,
            last_obs=None,
            observation_time=0.0,
        )
    )
    setpoints, plant_states, robot_states, _ = rerun.log_plant_tracking.call_args.args
    assert setpoints.joints[MG_ID][0] == [0.5, 0.5, 0.5]
    assert plant_states[MG_ID].joints == (0.25, 0.25, 0.25)
    assert robot_states[MG_ID].joints == (1.0, 2.0, 3.0)
    rerun.log_target_tracking.assert_not_called()
