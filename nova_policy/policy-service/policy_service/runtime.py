from __future__ import annotations

import asyncio
from dataclasses import dataclass
from datetime import UTC, datetime
import logging
from typing import TYPE_CHECKING
from uuid import uuid4

from .models import AppState, PolicyRunResponse, RunState

if TYPE_CHECKING:
    from .models import JsonValue, PolicyStartRequest

logger = logging.getLogger(__name__)


class PolicyConflictError(RuntimeError):
    """Raised when a run is already active."""


@dataclass(slots=True)
class _RunRecord:
    run_id: str
    policy: str
    timeout_s: float
    start_time: datetime
    state: RunState
    metadata: dict[str, JsonValue] | None
    stop_requested: bool = False


class MockPolicyRuntime:
    def __init__(self) -> None:
        self._lock = asyncio.Lock()
        self._app_state = AppState.EMPTY
        self._loaded_policy: str | None = None
        self._active_run_id: str | None = None
        self._runs: dict[str, _RunRecord] = {}
        self._tasks: set[asyncio.Task[None]] = set()

    @property
    def app_state(self) -> AppState:
        return self._app_state

    @property
    def loaded_policy(self) -> str | None:
        return self._loaded_policy

    def list_policies(self) -> list[str]:
        if self._loaded_policy is None:
            return []
        return [self._loaded_policy]

    async def start(self, policy: str, request: PolicyStartRequest) -> PolicyRunResponse:
        logger.info(
            "Policy start requested: policy=%s, params=%s",
            policy,
            request.model_dump(mode="json", exclude_none=False),
        )
        async with self._lock:
            if self._active_run_id is not None:
                active = self._runs[self._active_run_id]
                if active.state in {RunState.PREPARING, RunState.RUNNING, RunState.STOPPING}:
                    raise PolicyConflictError("A policy run is already active")

            if self._loaded_policy is None:
                self._app_state = AppState.LOADING
            elif self._loaded_policy != policy:
                self._app_state = AppState.SWITCHING
            self._loaded_policy = policy

            run_id = f"run_{uuid4().hex[:10]}"
            record = _RunRecord(
                run_id=run_id,
                policy=policy,
                timeout_s=request.timeout_s,
                start_time=datetime.now(UTC),
                state=RunState.PREPARING,
                metadata={"stage": "downloading", "percent": 5, "message": "Preparing policy"},
            )
            self._runs[run_id] = record
            self._active_run_id = run_id
            self._app_state = AppState.RUNNING

            task = asyncio.create_task(self._simulate_run(run_id), name=f"mock-policy-run-{run_id}")
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
            record.metadata = {"stage": "stopping", "percent": 100, "message": "Stopping run"}

    async def get_run(self, policy: str, run_id: str) -> PolicyRunResponse:
        async with self._lock:
            record = self._runs[run_id]
            if record.policy != policy:
                raise KeyError(run_id)
            return self._to_response(record)

    async def _simulate_run(self, run_id: str) -> None:
        await self._set_preparing_stage(run_id, "downloading", 30, "Downloading policy")
        await self._set_preparing_stage(run_id, "loading", 70, "Loading policy")
        await self._set_preparing_stage(run_id, "warming", 95, "Warming model")

        async with self._lock:
            record = self._runs[run_id]
            if record.stop_requested:
                self._finish_stopped(record)
                return
            record.state = RunState.RUNNING
            record.metadata = {"control_fps": 30.0, "inference_latency_ms_p50": 25.0}
            self._app_state = AppState.RUNNING

        deadline = asyncio.get_running_loop().time() + self._runs[run_id].timeout_s
        while asyncio.get_running_loop().time() < deadline:
            async with self._lock:
                record = self._runs[run_id]
                if record.stop_requested:
                    self._finish_stopped(record)
                    return
            await asyncio.sleep(0.1)

        async with self._lock:
            record = self._runs[run_id]
            if record.stop_requested:
                self._finish_stopped(record)
                return
            record.state = RunState.TIMED_OUT
            record.metadata = {"stage": "done", "message": "Timeout reached"}
            self._active_run_id = None
            self._app_state = AppState.READY

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
