from __future__ import annotations

import asyncio
import contextlib
from collections import deque
from dataclasses import dataclass
from typing import TYPE_CHECKING, Protocol, cast, runtime_checkable

from nova import api
from nova.cell.motion_group import MotionGroup

from .adapters import adapter_for_policy
from .client import PolicyServiceClient
from .models import ACTPolicy, ActionChunk, ActionStep, PolicyRun, RobotStatePoint

if TYPE_CHECKING:
    from collections.abc import AsyncIterator, Awaitable, Callable

    from .models import JsonValue


@runtime_checkable
class _HasModelDump(Protocol):
    def model_dump(self, *, mode: str, exclude_none: bool) -> dict[str, JsonValue]: ...


@dataclass(slots=True)
class PolicyExecutionContext:
    cameras: dict[str, dict[str, object]] | None = None


@dataclass(slots=True)
class PolicyExecutionOptions:
    tcp: str | None = None
    n_action_steps: int | None = None
    device: str | None = None
    cameras: dict[str, dict[str, object]] | None = None
    use_gripper: bool | None = None
    gripper_io_key: str | None = None
    motion_group_setup: _HasModelDump | None = None
    policy_api_url: str | None = None
    realtime: bool = False
    execute_actions: bool = False
    low_water_mark: int = 1
    max_observations: int | None = None
    allow_mock_images: bool = False
    joint_velocity_limit: float = 1.5
    joint_position_gain: float = 3.0
    joint_position_tolerance: float = 0.01


class _ActionStepQueue:
    def __init__(self) -> None:
        self._steps: deque[ActionStep] = deque()
        self._last_step: ActionStep | None = None

    def enqueue(self, steps: list[ActionStep]) -> None:
        self._steps.extend(steps)

    def should_request_chunk(self, low_water_mark: int) -> bool:
        return len(self._steps) <= low_water_mark

    def pop_or_hold(self) -> ActionStep | None:
        if self._steps:
            self._last_step = self._steps.popleft()
            return self._last_step
        return self._last_step

    @property
    def queued_steps(self) -> int:
        return len(self._steps)


class PolicyRunState:
    def __init__(self, run: PolicyRun, stop: Callable[[], Awaitable[None]]) -> None:
        self._run = run
        self._stop = stop

    @property
    def run(self) -> str:
        return self._run.run

    @property
    def policy(self) -> str:
        return self._run.policy

    @property
    def state(self) -> str:
        return self._run.state

    @property
    def elapsed_s(self) -> float | None:
        return self._run.elapsed_s

    @property
    def timeout_s(self) -> float | None:
        return self._run.timeout_s

    @property
    def metadata(self) -> dict[str, JsonValue] | None:
        return self._run.metadata

    async def stop(self) -> None:
        await self._stop()


def _to_json_payload(value: object) -> JsonValue:
    if value is None or isinstance(value, (str, int, float, bool)):
        return value

    if isinstance(value, _HasModelDump):
        return value.model_dump(mode="json", exclude_none=True)

    if isinstance(value, dict):
        result: dict[str, JsonValue] = {}
        value_dict = cast("dict[object, object]", value)
        for key, inner in value_dict.items():
            if inner is None:
                continue
            result[str(key)] = _to_json_payload(inner)
        return result

    if isinstance(value, list):
        inner_values = cast("list[object]", value)
        return [_to_json_payload(inner) for inner in inner_values]

    return str(value)


class _ApiConfig(Protocol):
    host: str
    access_token: str | None


class _ApiClientWithConfig(Protocol):
    config: _ApiConfig


class _ApiClientWithJogging(_ApiClientWithConfig, Protocol):
    motion_group_jogging_api: api.api.JoggingApi


def _api_config_from_motion_group(motion_group: MotionGroup) -> _ApiConfig:
    api_client_obj = cast("_ApiClientWithConfig", getattr(motion_group, "_api_client"))  # noqa: B009
    return api_client_obj.config


def _resolve_policy_client(motion_group: MotionGroup, policy_api_url: str | None) -> PolicyServiceClient:
    config = _api_config_from_motion_group(motion_group)
    base_url = policy_api_url or config.host
    return PolicyServiceClient(base_url=base_url, access_token=config.access_token)


def _controller_name(motion_group: MotionGroup) -> str:
    return cast("str", getattr(motion_group, "_controller_id"))  # noqa: B009


def _validate_execution_options(options: PolicyExecutionOptions) -> None:
    if options.execute_actions and not options.realtime:
        raise ValueError("execute_actions=True requires realtime=True")
    if options.low_water_mark < 0:
        raise ValueError("low_water_mark must be >= 0")
    if options.max_observations is not None and options.max_observations <= 0:
        raise ValueError("max_observations must be > 0 when provided")
    if options.joint_velocity_limit <= 0:
        raise ValueError("joint_velocity_limit must be > 0")
    if options.joint_position_gain <= 0:
        raise ValueError("joint_position_gain must be > 0")
    if options.joint_position_tolerance < 0:
        raise ValueError("joint_position_tolerance must be >= 0")


def _resolve_policy_spec(
    *,
    policy: ACTPolicy | None,
    policy_path: str | None,
    options: PolicyExecutionOptions,
) -> ACTPolicy:
    if policy is None and not policy_path:
        raise ValueError("Either policy or policy_path must be provided")

    if policy is not None and policy_path and policy.path != policy_path:
        raise ValueError("policy.path and policy_path must match when both are provided")

    option_steps = options.n_action_steps

    if policy is None:
        return ACTPolicy(path=cast("str", policy_path), n_action_steps=option_steps)

    if option_steps is None:
        return policy

    if policy.n_action_steps is None:
        return ACTPolicy(path=policy.path, n_action_steps=option_steps)

    if option_steps != policy.n_action_steps:
        raise ValueError("n_action_steps differs between policy and options")

    return policy


def _resolve_cameras(
    options: PolicyExecutionOptions,
    context: PolicyExecutionContext | None,
) -> dict[str, dict[str, object]] | None:
    if context is not None and context.cameras is not None:
        return context.cameras
    return options.cameras


async def _read_robot_state_point(
    motion_group: MotionGroup,
    *,
    tcp: str,
    use_gripper: bool | None,
) -> RobotStatePoint:
    state = await motion_group.get_state(tcp=tcp)
    joints = {f"joint_{index + 1}.pos": float(value) for index, value in enumerate(state.joints)}
    gripper = {"gripper.pos": 0.0} if use_gripper else None
    return RobotStatePoint(joints=joints, gripper=gripper)


def _compute_joint_velocity_towards_target(
    current: tuple[float, ...],
    target: tuple[float, ...],
    *,
    gain: float,
    velocity_limit: float,
    tolerance: float,
) -> tuple[float, ...]:
    velocities: list[float] = []
    for current_value, target_value in zip(current, target, strict=True):
        error = target_value - current_value
        if abs(error) <= tolerance:
            velocities.append(0.0)
            continue
        raw_velocity = error * gain
        velocities.append(max(-velocity_limit, min(velocity_limit, raw_velocity)))
    return tuple(velocities)


def _target_joints_from_step(step: ActionStep, current: tuple[float, ...]) -> tuple[float, ...]:
    targets: list[float] = []
    for index, current_value in enumerate(current):
        key = f"joint_{index + 1}.pos"
        targets.append(float(step.joints.get(key, current_value)))
    return tuple(targets)


class _JointJoggingSession:
    def __init__(self, motion_group: MotionGroup, *, tcp: str) -> None:
        self._motion_group = motion_group
        self._tcp = tcp
        self._commands: asyncio.Queue[tuple[float, ...] | None] = asyncio.Queue()
        self._task: asyncio.Task[None] | None = None
        self._closed = False

    async def command(self, velocity: tuple[float, ...]) -> None:
        if self._closed:
            raise RuntimeError("Cannot send joint jogging command after session was closed")
        if self._task is None:
            self._task = asyncio.create_task(self._run())
        if self._task.done():
            self._task.result()
        await self._commands.put(velocity)

    async def close(self) -> None:
        self._closed = True
        if self._task is None:
            return
        await self._commands.put(None)
        await self._task

    async def _run(self) -> None:
        api_client_obj = cast(
            "_ApiClientWithJogging", getattr(self._motion_group, "_api_client")  # noqa: B009
        )
        controller_id = cast("str", getattr(self._motion_group, "_controller_id"))  # noqa: B009
        cell = cast("str", getattr(self._motion_group, "_cell"))  # noqa: B009

        await api_client_obj.motion_group_jogging_api.execute_jogging(
            cell=cell,
            controller=controller_id,
            client_request_generator=self._request_generator,
        )

    async def _request_generator(self, response_stream):
        response_drain = asyncio.create_task(_drain_jogging_responses(response_stream))
        try:
            yield _initialize_jogging_request(self._motion_group.id, self._tcp)
            while True:
                velocity = await self._commands.get()
                if velocity is None:
                    break
                yield _joint_velocity_request(velocity)
            yield _pause_jogging_request()
        finally:
            response_drain.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await response_drain


async def _drain_jogging_responses(response_stream) -> None:
    async for _response in response_stream:
        pass


def _initialize_jogging_request(motion_group_id: str, tcp: str):
    return api.models.ExecuteJoggingRequest(
        root=api.models.InitializeJoggingRequest(motion_group=motion_group_id, tcp=tcp)
    )


def _joint_velocity_request(velocity: tuple[float, ...]):
    return api.models.ExecuteJoggingRequest(
        root=api.models.JointVelocityRequest(velocity=api.models.Joints(root=list(velocity)))
    )


def _pause_jogging_request():
    return api.models.ExecuteJoggingRequest(root=api.models.PauseJoggingRequest())


async def _apply_action_step(
    motion_group: MotionGroup,
    step: ActionStep,
    *,
    jogging_session: _JointJoggingSession,
    options: PolicyExecutionOptions,
) -> None:
    current = await motion_group.joints()
    target = _target_joints_from_step(step, current)
    velocity = _compute_joint_velocity_towards_target(
        current,
        target,
        gain=options.joint_position_gain,
        velocity_limit=options.joint_velocity_limit,
        tolerance=options.joint_position_tolerance,
    )
    await jogging_session.command(velocity)


def _action_step_to_metadata(step: ActionStep) -> dict[str, JsonValue]:
    metadata: dict[str, JsonValue] = {"joints": dict(step.joints)}
    if step.gripper is not None:
        metadata["gripper"] = dict(step.gripper)
    if step.io is not None:
        metadata["io"] = dict(step.io)
    return metadata


def _chunk_to_metadata(chunk: ActionChunk) -> dict[str, JsonValue]:
    return {
        "chunk_id": chunk.chunk_id,
        "observation_seq": chunk.observation_seq,
        "n_action_steps": chunk.n_action_steps,
        "control_dt_s": chunk.control_dt_s,
        "inference_latency_ms": chunk.inference_latency_ms,
    }


def _with_realtime_metadata(
    run: PolicyRun,
    *,
    observation_seq: int,
    queued_action_steps: int,
    last_chunk: ActionChunk | None,
    last_action_step: ActionStep | None,
) -> PolicyRun:
    metadata = dict(run.metadata or {})
    realtime_metadata: dict[str, JsonValue] = {
        "next_observation_seq": observation_seq,
        "last_observation_seq": observation_seq - 1 if observation_seq > 0 else None,
        "queued_action_steps": queued_action_steps,
    }
    if last_chunk is not None:
        realtime_metadata["last_action_chunk"] = _chunk_to_metadata(last_chunk)
    if last_action_step is not None:
        realtime_metadata["last_action_step"] = _action_step_to_metadata(last_action_step)
    metadata["realtime"] = realtime_metadata
    return PolicyRun(
        run=run.run,
        policy=run.policy,
        state=run.state,
        start_time=run.start_time,
        timeout_s=run.timeout_s,
        elapsed_s=run.elapsed_s,
        metadata=metadata,
    )


def _control_dt_s_from_metadata(metadata: dict[str, JsonValue] | None) -> float | None:
    if not isinstance(metadata, dict):
        return None
    value = metadata.get("control_dt_s")
    if not isinstance(value, (int, float)) or value <= 0:
        return None
    return float(value)


async def _run_realtime_policy_loop(  # noqa: PLR0913
    motion_group: MotionGroup,
    *,
    client: PolicyServiceClient,
    policy_path: str,
    run_id: str,
    task: str,
    tcp: str,
    options: PolicyExecutionOptions,
    stop_run: Callable[[], Awaitable[None]],
) -> AsyncIterator[PolicyRunState]:
    realtime = client.open_realtime_session()
    jogging_session = _JointJoggingSession(motion_group, tcp=tcp) if options.execute_actions else None
    action_queue = _ActionStepQueue()
    observation_seq = 0
    last_chunk: ActionChunk | None = None
    last_action_step: ActionStep | None = None
    stop_requested = False

    try:
        while True:
            status = await client.get_run(policy=policy_path, run=run_id)
            yield PolicyRunState(
                run=_with_realtime_metadata(
                    status,
                    observation_seq=observation_seq,
                    queued_action_steps=action_queue.queued_steps,
                    last_chunk=last_chunk,
                    last_action_step=last_action_step,
                ),
                stop=stop_run,
            )
            if status.state in {"STOPPED", "TIMED_OUT", "FAILED"}:
                return

            if status.state != "RUNNING":
                await asyncio.sleep(client.status_poll_interval_s)
                continue

            if options.execute_actions:
                step = action_queue.pop_or_hold()
                if step is not None:
                    if jogging_session is None:
                        raise RuntimeError("Jogging session is required when execute_actions=True")
                    await _apply_action_step(
                        motion_group,
                        step,
                        jogging_session=jogging_session,
                        options=options,
                    )
                    last_action_step = step

            if action_queue.should_request_chunk(options.low_water_mark):
                state_point = await _read_robot_state_point(
                    motion_group,
                    tcp=tcp,
                    use_gripper=options.use_gripper,
                )
                chunk = await realtime.predict(
                    run=run_id,
                    seq=observation_seq,
                    state_point=state_point,
                    task=task,
                )
                action_queue.enqueue(chunk.steps)
                last_chunk = chunk
                observation_seq += 1

                if options.max_observations is not None and observation_seq >= options.max_observations:
                    await stop_run()
                    stop_requested = True

            sleep_s = client.status_poll_interval_s if stop_requested else (
                _control_dt_s_from_metadata(status.metadata) or client.status_poll_interval_s
            )
            await asyncio.sleep(sleep_s)
    finally:
        if jogging_session is not None:
            await jogging_session.close()
        await realtime.close()


async def execute_policy(  # noqa: PLR0913
    self: MotionGroup,
    policy_path: str | None = None,
    task: str = "",
    timeout_s: float = 120.0,
    *,
    policy: ACTPolicy | None = None,
    options: PolicyExecutionOptions | None = None,
    context: PolicyExecutionContext | None = None,
) -> PolicyRunState:
    selected_options = options or PolicyExecutionOptions()
    last_state: PolicyRunState | None = None
    async for state in stream_policy(
        self,
        policy_path=policy_path,
        policy=policy,
        task=task,
        timeout_s=timeout_s,
        options=selected_options,
        context=context,
    ):
        last_state = state

    if last_state is None:
        raise RuntimeError("Policy stream ended without a state")

    return last_state


async def stream_policy(  # noqa: PLR0913
    self: MotionGroup,
    policy_path: str | None = None,
    task: str = "",
    timeout_s: float = 120.0,
    *,
    policy: ACTPolicy | None = None,
    options: PolicyExecutionOptions | None = None,
    context: PolicyExecutionContext | None = None,
) -> AsyncIterator[PolicyRunState]:
    selected_options = options or PolicyExecutionOptions()
    _validate_execution_options(selected_options)
    resolved_policy = _resolve_policy_spec(
        policy=policy,
        policy_path=policy_path,
        options=selected_options,
    )
    client = _resolve_policy_client(self, selected_options.policy_api_url)

    resolved_tcp = selected_options.tcp or await self.active_tcp_name() or "flange"
    motion_group_setup = selected_options.motion_group_setup
    if motion_group_setup is None:
        setup_obj = await self.get_setup(resolved_tcp)
        motion_group_setup = cast("_HasModelDump", setup_obj)

    adapter = adapter_for_policy(resolved_policy)
    payload = {
        "policy": adapter.service_policy_payload(device=selected_options.device),
        "target": {
            "controller_name": _controller_name(self),
            "motion_group": self.id,
            "tcp": resolved_tcp,
        },
        "task": task,
        "timeout_s": timeout_s,
        "cameras": _resolve_cameras(selected_options, context),
        "gripper": {
            "use_gripper": selected_options.use_gripper,
            "gripper_io_key": selected_options.gripper_io_key,
        },
        "motion_group_setup": _to_json_payload(motion_group_setup),
        "allow_mock_images": selected_options.allow_mock_images,
    }
    payload_json = cast("dict[str, object]", _to_json_payload(payload))

    run = await client.start_run(policy=resolved_policy.path, payload=payload_json)

    async def stop_run() -> None:
        await client.stop_run(policy=resolved_policy.path, run=run.run)

    yield PolicyRunState(run=run, stop=stop_run)

    if selected_options.realtime:
        async for status in _run_realtime_policy_loop(
            self,
            client=client,
            policy_path=resolved_policy.path,
            run_id=run.run,
            task=task,
            tcp=resolved_tcp,
            options=selected_options,
            stop_run=stop_run,
        ):
            yield status
        return

    async for status in client.stream_run(policy=resolved_policy.path, run=run.run):
        yield PolicyRunState(run=status, stop=stop_run)


def enable_motion_group_policy_extension() -> None:
    setattr(MotionGroup, "execute_policy", execute_policy)  # noqa: B010
    setattr(MotionGroup, "stream_policy", stream_policy)  # noqa: B010


enable_motion_group_policy_extension()
