from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Protocol, cast, runtime_checkable

from nova.cell.motion_group import MotionGroup

from .client import NovaLeRobotPolicyClient

if TYPE_CHECKING:
    from collections.abc import AsyncIterator, Awaitable, Callable

    from .models import JsonValue, PolicyRun


@runtime_checkable
class _HasModelDump(Protocol):
    def model_dump(self, *, mode: str, exclude_none: bool) -> dict[str, JsonValue]: ...


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


def _api_config_from_motion_group(motion_group: MotionGroup) -> _ApiConfig:
    api_client_obj = cast("_ApiClientWithConfig", getattr(motion_group, "_api_client"))  # noqa: B009
    return api_client_obj.config


def _resolve_policy_client(
    motion_group: MotionGroup, policy_api_url: str | None
) -> NovaLeRobotPolicyClient:
    config = _api_config_from_motion_group(motion_group)
    base_url = policy_api_url or config.host
    return NovaLeRobotPolicyClient(base_url=base_url, access_token=config.access_token)


def _controller_name(motion_group: MotionGroup) -> str:
    return cast("str", getattr(motion_group, "_controller_id"))  # noqa: B009


async def execute_policy(
    self: MotionGroup,
    policy_path: str,
    task: str,
    timeout_s: float,
    *,
    options: PolicyExecutionOptions | None = None,
) -> PolicyRunState:
    selected_options = options or PolicyExecutionOptions()
    last_state: PolicyRunState | None = None
    async for state in stream_policy(
        self,
        policy_path=policy_path,
        task=task,
        timeout_s=timeout_s,
        options=selected_options,
    ):
        last_state = state

    if last_state is None:
        raise RuntimeError("Policy stream ended without a state")

    return last_state


async def stream_policy(
    self: MotionGroup,
    policy_path: str,
    task: str,
    timeout_s: float,
    *,
    options: PolicyExecutionOptions | None = None,
) -> AsyncIterator[PolicyRunState]:
    selected_options = options or PolicyExecutionOptions()
    client = _resolve_policy_client(self, selected_options.policy_api_url)

    resolved_tcp = selected_options.tcp or await self.active_tcp_name() or "flange"
    motion_group_setup = selected_options.motion_group_setup
    if motion_group_setup is None:
        setup_obj = await self.get_setup(resolved_tcp)
        motion_group_setup = cast("_HasModelDump", setup_obj)

    payload = {
        "policy": {
            "path": policy_path,
            "n_action_steps": selected_options.n_action_steps,
            "device": selected_options.device,
        },
        "target": {
            "controller_name": _controller_name(self),
            "motion_group": self.id,
            "tcp": resolved_tcp,
        },
        "task": task,
        "timeout_s": timeout_s,
        "cameras": selected_options.cameras,
        "gripper": {
            "use_gripper": selected_options.use_gripper,
            "gripper_io_key": selected_options.gripper_io_key,
        },
        "motion_group_setup": _to_json_payload(motion_group_setup),
    }
    payload_json = cast("dict[str, object]", _to_json_payload(payload))

    run = await client.start_run(policy=policy_path, payload=payload_json)

    async def stop_run() -> None:
        await client.stop_run(policy=policy_path, run=run.run)

    yield PolicyRunState(run=run, stop=stop_run)

    async for status in client.stream_run(policy=policy_path, run=run.run):
        yield PolicyRunState(run=status, stop=stop_run)


def enable_motion_group_policy_extension() -> None:
    setattr(MotionGroup, "execute_policy", execute_policy)  # noqa: B010
    setattr(MotionGroup, "stream_policy", stream_policy)  # noqa: B010


enable_motion_group_policy_extension()
