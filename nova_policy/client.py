from __future__ import annotations

import asyncio
import importlib
from typing import TYPE_CHECKING, cast
from urllib.parse import quote

import httpx

from .models import ActionChunk, ActionStep, ACTPolicy, PolicyRun

if TYPE_CHECKING:
    from collections.abc import AsyncIterator, Mapping

    from .models import JsonValue, PolicyRunPayload, PolicySpec, RobotStatePoint
HTTP_OK_STATUS = 200
HTTP_CONFLICT_STATUS = 406
HTTP_NO_CONTENT_STATUS = 204

TERMINAL_POLICY_RUN_STATES = {"STOPPED", "TIMED_OUT", "FAILED"}


def _is_terminal_state(state: str) -> bool:
    return state in TERMINAL_POLICY_RUN_STATES


class PolicyConflictError(RuntimeError):
    """Raised when a policy run is already active for the requested scope."""


def _load_socketio_module() -> object:
    try:
        return importlib.import_module("socketio")
    except ModuleNotFoundError as exc:  # pragma: no cover - dependency configuration issue
        raise RuntimeError(
            "python-socketio is required for realtime policy inference. "
            "Install the wandelbots-nova 'nova-policy' extra."
        ) from exc


class PolicyRealtimeSession:
    def __init__(
        self,
        *,
        base_url: str,
        headers: Mapping[str, str],
        timeout_s: float,
        reconnection_attempts: int,
        reconnection_delay_s: float,
        reconnection_delay_max_s: float,
    ) -> None:
        self._base_url = base_url.rstrip("/")
        self._headers = dict(headers)
        self._timeout_s = timeout_s
        self._socketio_module = _load_socketio_module()
        self._client = self._socketio_module.AsyncClient(
            reconnection=True,
            reconnection_attempts=reconnection_attempts,
            reconnection_delay=reconnection_delay_s,
            reconnection_delay_max=reconnection_delay_max_s,
        )
        self._pending_chunks: dict[tuple[str, int], asyncio.Future[ActionChunk]] = {}
        self._latest_session_state: dict[str, object] | None = None
        self._install_handlers()

    @property
    def latest_session_state(self) -> dict[str, object] | None:
        return self._latest_session_state

    async def connect(self) -> None:
        if self._client.connected:
            return
        await self._client.connect(
            self._base_url,
            headers=self._headers,
            socketio_path="socket.io",
            wait_timeout=self._timeout_s,
        )

    async def close(self) -> None:
        if not self._client.connected:
            return
        await self._client.disconnect()

    async def predict(
        self,
        *,
        run: str,
        seq: int,
        state_point: RobotStatePoint,
        task: str | None = None,
        timeout_s: float | None = None,
    ) -> ActionChunk:
        await self.connect()

        effective_timeout_s = timeout_s or self._timeout_s
        loop = asyncio.get_running_loop()
        future: asyncio.Future[ActionChunk] = loop.create_future()
        pending_key = (run, seq)
        self._pending_chunks[pending_key] = future

        try:
            ack = await self._client.call(
                "observation.push",
                {
                    "run": run,
                    "seq": seq,
                    "task": task,
                    "observation": state_point.to_observation(),
                },
                timeout=effective_timeout_s,
            )
            if not isinstance(ack, dict) or not bool(ack.get("accepted")):
                raise RuntimeError(str(ack.get("error", "Observation was not accepted")))
            return await asyncio.wait_for(future, timeout=effective_timeout_s)
        finally:
            self._pending_chunks.pop(pending_key, None)

    def _install_handlers(self) -> None:
        @self._client.on("action.chunk")
        def _on_action_chunk(payload: object) -> None:
            if not isinstance(payload, dict):
                return
            chunk = PolicyServiceClient._parse_action_chunk_payload(payload)
            future = self._pending_chunks.get((chunk.run, chunk.observation_seq))
            if future is None or future.done():
                return
            future.set_result(chunk)

        @self._client.on("session.state")
        def _on_session_state(payload: object) -> None:
            if isinstance(payload, dict):
                self._latest_session_state = cast("dict[str, object]", payload)


class PolicyServiceClient:
    def __init__(
        self,
        base_url: str,
        access_token: str | None = None,
        timeout_s: float = 60.0,
        status_poll_interval_s: float = 0.25,
        realtime_reconnection_attempts: int = 5,
        realtime_reconnection_delay_s: float = 0.5,
        realtime_reconnection_delay_max_s: float = 5.0,
    ) -> None:
        if realtime_reconnection_attempts < 0:
            raise ValueError("realtime_reconnection_attempts must be >= 0")
        if realtime_reconnection_delay_s <= 0 or realtime_reconnection_delay_max_s <= 0:
            raise ValueError("realtime reconnection delays must be > 0")

        self._base_url = base_url.rstrip("/")
        self._timeout_s = timeout_s
        self._status_poll_interval_s = status_poll_interval_s
        self._realtime_reconnection_attempts = realtime_reconnection_attempts
        self._realtime_reconnection_delay_s = realtime_reconnection_delay_s
        self._realtime_reconnection_delay_max_s = realtime_reconnection_delay_max_s
        self._headers = {"Authorization": f"Bearer {access_token}"} if access_token else {}

    @property
    def status_poll_interval_s(self) -> float:
        return self._status_poll_interval_s

    async def get_policy(self) -> PolicySpec:
        payload = await self._request_json(
            "GET",
            "/policy",
            expected_status={HTTP_OK_STATUS},
        )
        return self._parse_policy_payload(payload)

    async def start_run(self, policy: str, payload: Mapping[str, object]) -> PolicyRun:
        policy_id = quote(policy, safe="")
        response = await self._request_json(
            "POST",
            f"/policies/{policy_id}/start",
            json_body=payload,
            expected_status={HTTP_OK_STATUS},
        )
        return PolicyRun.from_dict(self._parse_policy_run_payload(response))

    async def stop_run(self, policy: str, run: str | None = None) -> None:
        policy_id = quote(policy, safe="")
        params = {"run": run} if run else None
        await self._request_json(
            "POST",
            f"/policies/{policy_id}/stop",
            params=params,
            expected_status={HTTP_NO_CONTENT_STATUS},
        )

    async def get_run(self, policy: str, run: str) -> PolicyRun:
        policy_id = quote(policy, safe="")
        response = await self._request_json(
            "GET",
            f"/policies/{policy_id}/runs/{run}",
            expected_status={HTTP_OK_STATUS},
        )
        return PolicyRun.from_dict(self._parse_policy_run_payload(response))

    async def stream_run(self, policy: str, run: str) -> AsyncIterator[PolicyRun]:
        while True:
            status = await self.get_run(policy=policy, run=run)
            yield status
            if _is_terminal_state(status.state):
                return
            await asyncio.sleep(self._status_poll_interval_s)

    def open_realtime_session(self) -> PolicyRealtimeSession:
        return PolicyRealtimeSession(
            base_url=self._base_url,
            headers=self._headers,
            timeout_s=self._timeout_s,
            reconnection_attempts=self._realtime_reconnection_attempts,
            reconnection_delay_s=self._realtime_reconnection_delay_s,
            reconnection_delay_max_s=self._realtime_reconnection_delay_max_s,
        )

    @staticmethod
    def _parse_policy_payload(payload: dict[str, object]) -> PolicySpec:
        kind = payload.get("kind")
        path = payload.get("path")
        if not isinstance(kind, str) or not isinstance(path, str) or not path:
            raise ValueError("Policy response must contain non-empty string 'kind' and 'path'")

        normalized_kind = kind.lower()
        if normalized_kind == "act":
            return ACTPolicy(path=path)

        raise ValueError(f"Unsupported policy kind '{kind}'")

    @staticmethod
    def _parse_policy_run_payload(payload: dict[str, object]) -> PolicyRunPayload:
        run = payload.get("run")
        state = payload.get("state")
        if not isinstance(run, str) or not isinstance(state, str):
            raise ValueError("Policy service response must contain string 'run' and 'state'")

        policy_value = payload.get("policy")
        policy_name = policy_value if isinstance(policy_value, str) else ""

        parsed: PolicyRunPayload = {
            "run": run,
            "state": state,
            "policy": policy_name,
        }

        start_time = payload.get("start_time")
        if isinstance(start_time, str):
            parsed["start_time"] = start_time

        timeout_s = payload.get("timeout_s")
        if isinstance(timeout_s, (int, float)):
            parsed["timeout_s"] = float(timeout_s)

        elapsed_s = payload.get("elapsed_s")
        if isinstance(elapsed_s, (int, float)):
            parsed["elapsed_s"] = float(elapsed_s)

        metadata = payload.get("metadata")
        if isinstance(metadata, dict):
            parsed["metadata"] = cast("dict[str, JsonValue]", metadata)

        return parsed

    @staticmethod
    def _parse_action_chunk_payload(payload: dict[str, object]) -> ActionChunk:
        def _as_float(value: object, field_name: str) -> float:
            if not isinstance(value, (int, float)):
                raise ValueError(f"Action chunk field '{field_name}' must be numeric")
            return float(value)

        def _parse_step(step_payload: object) -> ActionStep:
            if not isinstance(step_payload, dict):
                raise ValueError("Action chunk steps must contain objects")

            joints_payload = step_payload.get("joints")
            if not isinstance(joints_payload, dict):
                raise ValueError("Action step must contain object 'joints'")
            joints = {
                str(key): _as_float(value, f"steps[].joints.{key}")
                for key, value in joints_payload.items()
            }

            gripper_payload = step_payload.get("gripper")
            gripper = None
            if isinstance(gripper_payload, dict):
                gripper = {
                    str(key): _as_float(value, f"steps[].gripper.{key}")
                    for key, value in gripper_payload.items()
                }

            io_payload = step_payload.get("io")
            io = cast("dict[str, JsonValue] | None", io_payload) if isinstance(io_payload, dict) else None
            return ActionStep(joints=joints, gripper=gripper, io=io)

        run = payload.get("run")
        policy = payload.get("policy")
        policy_kind = payload.get("policy_kind")
        chunk_id = payload.get("chunk_id")
        observation_seq = payload.get("observation_seq")
        n_action_steps = payload.get("n_action_steps")
        if not all(isinstance(value, str) for value in [run, policy, policy_kind, chunk_id]):
            raise ValueError("Action chunk payload is missing string identity fields")
        if not isinstance(observation_seq, int) or not isinstance(n_action_steps, int):
            raise ValueError("Action chunk payload is missing integer sequencing fields")

        steps_payload = payload.get("steps")
        if not isinstance(steps_payload, list):
            raise ValueError("Action chunk payload must contain list 'steps'")

        diagnostics_payload = payload.get("diagnostics")
        diagnostics = (
            cast("dict[str, JsonValue]", diagnostics_payload)
            if isinstance(diagnostics_payload, dict)
            else None
        )

        model_time = payload.get("model_time")
        first_step_at_s = payload.get("first_step_at_s")
        return ActionChunk(
            run=run,
            policy=policy,
            policy_kind=policy_kind,
            chunk_id=chunk_id,
            observation_seq=observation_seq,
            n_action_steps=n_action_steps,
            control_dt_s=_as_float(payload.get("control_dt_s"), "control_dt_s"),
            inference_latency_ms=_as_float(
                payload.get("inference_latency_ms"),
                "inference_latency_ms",
            ),
            steps=[_parse_step(step_payload) for step_payload in steps_payload],
            model_time=float(model_time) if isinstance(model_time, (int, float)) else None,
            first_step_at_s=(
                float(first_step_at_s) if isinstance(first_step_at_s, (int, float)) else None
            ),
            diagnostics=diagnostics,
        )

    async def _request_json(
        self,
        method: str,
        path: str,
        *,
        expected_status: set[int],
        json_body: Mapping[str, object] | None = None,
        params: Mapping[str, str] | None = None,
    ) -> dict[str, object]:
        async with httpx.AsyncClient(
            base_url=self._base_url,
            headers=self._headers,
            timeout=self._timeout_s,
        ) as client:
            response = await client.request(method, path, json=json_body, params=params)

        if response.status_code == HTTP_CONFLICT_STATUS:
            raise PolicyConflictError(response.text)

        if response.status_code not in expected_status:
            response.raise_for_status()

        if response.status_code == HTTP_NO_CONTENT_STATUS:
            return {}

        payload_raw = cast("object", response.json())
        if not isinstance(payload_raw, dict):
            raise ValueError("Policy service response must be a JSON object")

        return cast("dict[str, object]", payload_raw)


class NovaLeRobotPolicyClient(PolicyServiceClient):
    """Backward-compatible alias for the old client name."""
