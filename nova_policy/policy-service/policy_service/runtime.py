from __future__ import annotations

import asyncio
from dataclasses import dataclass
from datetime import UTC, datetime
import logging
import os
from typing import TYPE_CHECKING
from uuid import uuid4

from .lerobot_inference_engine import LeRobotInferenceEngine
from .models import AppState, PolicyRunResponse, RunState

if TYPE_CHECKING:
    from .models import JsonValue, PolicyStartRequest

logger = logging.getLogger(__name__)


def _env_flag(name: str, *, default: bool = False) -> bool:
    value = os.getenv(name)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


class PolicyConflictError(RuntimeError):
    """Raised when a run is already active."""


@dataclass(slots=True)
class _RunRecord:
    run_id: str
    policy: str
    timeout_s: float
    device: str
    start_time: datetime
    state: RunState
    metadata: dict[str, JsonValue] | None
    stop_requested: bool = False
    task: str | None = None


class PolicyRuntime:
    def __init__(self) -> None:
        self._lock = asyncio.Lock()
        self._app_state = AppState.EMPTY
        self._loaded_policy: str | None = None
        self._active_run_id: str | None = None
        self._runs: dict[str, _RunRecord] = {}
        self._tasks: set[asyncio.Task[None]] = set()
        self._loader = LeRobotInferenceEngine()

        configured_policy_path = os.getenv("POLICY_PATH", "").strip()
        configured_policy_kind = os.getenv("POLICY_KIND", "act").strip().lower()
        configured_device = os.getenv("POLICY_DEVICE", "cuda").strip()

        self._configured_policy_path = configured_policy_path or None
        self._configured_policy_kind = configured_policy_kind or "act"
        self._policy_device = configured_device or "cuda"
        self._preload_policy_on_startup = _env_flag("PRELOAD_POLICY_ON_STARTUP", default=False)

    @property
    def app_state(self) -> AppState:
        return self._app_state

    @property
    def loaded_policy(self) -> str | None:
        return self._loaded_policy

    @property
    def configured_policy_path(self) -> str | None:
        return self._configured_policy_path

    @property
    def configured_policy_kind(self) -> str:
        return self._configured_policy_kind

    async def startup(self) -> None:
        if not self._preload_policy_on_startup:
            return

        if self._configured_policy_path is None:
            logger.warning("PRELOAD_POLICY_ON_STARTUP is true but POLICY_PATH is not configured")
            return

        logger.info("Preloading configured policy: %s", self._configured_policy_path)
        async with self._lock:
            self._app_state = AppState.LOADING

        try:
            await self._loader.ensure_loaded(
                policy_path=self._configured_policy_path,
                device=self._policy_device,
            )
        except Exception:
            logger.exception("Failed to preload configured policy")
            async with self._lock:
                self._app_state = AppState.ERROR
            return

        async with self._lock:
            self._loaded_policy = self._configured_policy_path
            self._app_state = AppState.READY

    def list_policies(self) -> list[str]:
        if self._configured_policy_path is None:
            return []
        return [self._configured_policy_path]

    async def start(self, policy: str, request: PolicyStartRequest) -> PolicyRunResponse:
        logger.info(
            "Policy start requested: policy=%s, params=%s",
            policy,
            request.model_dump(mode="json", exclude_none=False),
        )

        policy_path = self._resolve_policy_path(policy=policy, request=request)
        device = self._resolve_policy_device()

        async with self._lock:
            if self._active_run_id is not None:
                active = self._runs[self._active_run_id]
                if active.state in {RunState.PREPARING, RunState.RUNNING, RunState.STOPPING}:
                    raise PolicyConflictError("A policy run is already active")

            if self._loaded_policy is None:
                self._app_state = AppState.LOADING
            elif self._loaded_policy != policy_path:
                self._app_state = AppState.SWITCHING

            run_id = f"run_{uuid4().hex[:10]}"
            record = _RunRecord(
                run_id=run_id,
                policy=policy_path,
                timeout_s=request.timeout_s,
                device=device,
                start_time=datetime.now(UTC),
                state=RunState.PREPARING,
                metadata={"stage": "queued", "message": "Run accepted"},
                task=request.task,
            )
            self._runs[run_id] = record
            self._active_run_id = run_id

            task = asyncio.create_task(self._run_policy(run_id), name=f"policy-run-{run_id}")
            self._tasks.add(task)
            task.add_done_callback(self._tasks.discard)
            return self._to_response(record)

    async def stop(self, policy: str, run_id: str | None) -> None:
        async with self._lock:
            target_run_id = run_id or self._active_run_id
            if target_run_id is None:
                return

            record = self._runs.get(target_run_id)
            if record is None or record.policy != policy:
                return

            if record.state in {RunState.STOPPED, RunState.TIMED_OUT, RunState.FAILED}:
                return

            record.stop_requested = True
            record.state = RunState.STOPPING
            record.metadata = {"stage": "stopping", "message": "Stopping run"}

    async def get_run(self, policy: str, run_id: str) -> PolicyRunResponse:
        async with self._lock:
            record = self._runs[run_id]
            if record.policy != policy:
                raise KeyError(run_id)
            return self._to_response(record)

    async def _run_policy(self, run_id: str) -> None:
        await self._set_preparing_stage(run_id, "downloading", 20, "Resolving policy artifacts")
        await self._set_preparing_stage(run_id, "loading", 60, "Loading model")

        loaded = await self._prepare_loaded_policy(run_id)
        if not loaded:
            return

        try:
            task = self._runs[run_id].task
            warmup_latency_ms, warmup_joints = await self._loader.warmup(task=task)
        except Exception as exc:
            logger.exception("Failed during policy warmup inference")
            await self._mark_failed(run_id, f"Warmup inference failed: {exc}")
            return

        control_fps = self._loader.control_fps
        entered_running = await self._set_running_state(
            run_id,
            control_fps,
            warmup_latency_ms,
            warmup_joints,
        )
        if not entered_running:
            return

        await self._run_inference_loop_until_timeout(run_id, control_fps)

    async def _prepare_loaded_policy(self, run_id: str) -> bool:
        try:
            async with self._lock:
                record = self._runs[run_id]
                if record.stop_requested:
                    self._finish_stopped(record)
                    return False
                policy_path = record.policy
                device = record.device

            await self._loader.ensure_loaded(policy_path=policy_path, device=device)

            async with self._lock:
                self._loaded_policy = policy_path
                record = self._runs[run_id]
                if record.stop_requested:
                    self._finish_stopped(record)
                    return False
                record.metadata = {"stage": "warming", "percent": 95, "message": "Warmup model"}
                self._app_state = AppState.READY
            return True
        except Exception as exc:
            logger.exception("Failed to load policy")
            await self._mark_failed(run_id, str(exc))
            return False

    async def _set_running_state(
        self,
        run_id: str,
        control_fps: float,
        latency_ms: float,
        joint_values: list[float] | None,
    ) -> bool:
        async with self._lock:
            record = self._runs[run_id]
            if record.stop_requested:
                self._finish_stopped(record)
                return False
            record.state = RunState.RUNNING
            record.metadata = {
                "control_fps": control_fps,
                "inference_latency_ms_p50": round(latency_ms, 2),
                "last_joint_values": joint_values,
            }
            self._app_state = AppState.RUNNING
        return True

    async def _run_inference_loop_until_timeout(self, run_id: str, control_fps: float) -> None:
        deadline = asyncio.get_running_loop().time() + self._runs[run_id].timeout_s
        while asyncio.get_running_loop().time() < deadline:
            async with self._lock:
                record = self._runs[run_id]
                if record.stop_requested:
                    self._finish_stopped(record)
                    return

            try:
                task = self._runs[run_id].task
                latency_ms, joint_values = await self._loader.infer_once(task=task)
            except Exception as exc:
                logger.exception("Policy inference tick failed")
                await self._mark_failed(run_id, f"Inference failed: {exc}")
                return

            async with self._lock:
                record = self._runs[run_id]
                if record.state == RunState.RUNNING:
                    record.metadata = {
                        "control_fps": control_fps,
                        "inference_latency_ms_p50": round(latency_ms, 2),
                        "last_joint_values": joint_values,
                    }

            await asyncio.sleep(1.0 / control_fps)

        async with self._lock:
            record = self._runs[run_id]
            if record.stop_requested:
                self._finish_stopped(record)
                return
            record.state = RunState.TIMED_OUT
            record.metadata = {"stage": "done", "message": "Timeout reached"}
            self._active_run_id = None
            self._app_state = AppState.READY

    async def _mark_failed(self, run_id: str, message: str) -> None:
        async with self._lock:
            record = self._runs[run_id]
            record.state = RunState.FAILED
            record.metadata = {"stage": "failed", "message": message}
            self._active_run_id = None
            self._app_state = AppState.ERROR

    async def _set_preparing_stage(
        self, run_id: str, stage: str, percent: int, message: str
    ) -> None:
        async with self._lock:
            record = self._runs[run_id]
            if record.stop_requested:
                self._finish_stopped(record)
                return
            record.state = RunState.PREPARING
            record.metadata = {"stage": stage, "percent": percent, "message": message}
            self._app_state = AppState.LOADING
        await asyncio.sleep(0.2)

    def _finish_stopped(self, record: _RunRecord) -> None:
        record.state = RunState.STOPPED
        record.metadata = {"stage": "stopped", "message": "Stopped by request"}
        if self._active_run_id == record.run_id:
            self._active_run_id = None
        self._app_state = AppState.READY

    def _resolve_policy_path(self, policy: str, request: PolicyStartRequest) -> str:
        configured_path = self._configured_policy_path
        if configured_path is None:
            raise ValueError("POLICY_PATH must be configured on the service")

        requested_paths: list[str] = []
        if policy:
            requested_paths.append(policy)

        if request.policy is not None:
            request_policy_path = request.policy.get("path")
            if isinstance(request_policy_path, str) and request_policy_path:
                requested_paths.append(request_policy_path)

        for requested_path in requested_paths:
            if requested_path != configured_path:
                raise ValueError(
                    "Policy selection is environment-driven. "
                    "Requested policy does not match configured POLICY_PATH."
                )

        return configured_path

    def _resolve_policy_device(self) -> str:
        return self._policy_device

    @staticmethod
    def _to_response(record: _RunRecord) -> PolicyRunResponse:
        elapsed_s = (datetime.now(UTC) - record.start_time).total_seconds()
        return PolicyRunResponse(
            run=record.run_id,
            policy=record.policy,
            state=record.state,
            start_time=record.start_time.isoformat(),
            timeout_s=record.timeout_s,
            elapsed_s=elapsed_s,
            metadata=record.metadata,
        )
