"""PolicyExecutor — runs one policy episode via PID-controlled jogging."""

from __future__ import annotations

import asyncio
import contextlib
import logging
import time
from collections.abc import Callable, Coroutine
from dataclasses import dataclass
from enum import StrEnum
from typing import TYPE_CHECKING, Any

from policy.runner import PolicyRunner
from policy.types import ActionChunk, EmergencyStopError, GuardStopError, MotionError

if TYPE_CHECKING:
    from policy.cameras import CameraSet
    from policy.feature_map import FeatureMap
    from policy.policy_client import PolicyClient
    from policy.types import PolicyRunnerConfig, SafetyGuard

# Type for bare async policy functions
_PolicyFn = Callable[[dict[str, Any]], Coroutine[Any, Any, dict[str, float] | ActionChunk]]

logger = logging.getLogger(__name__)


class Phase(StrEnum):
    """Executor lifecycle phase."""

    IDLE = "IDLE"
    EXECUTING = "EXECUTING"


@dataclass
class ExecutorStatus:
    """Current executor state, queryable at any time."""

    phase: Phase = Phase.IDLE
    step: int = 0
    message: str = ""


@dataclass
class ExecutionResult:
    """Result of a policy execution run."""

    reason: str
    """Why execution ended: 'timeout' | 'stopped' | 'safety_guard' | 'estop' | 'error'"""

    steps: int = 0
    duration_s: float = 0.0
    last_state: dict[str, Any] | None = None
    """Last observed robot state (per motion group). Useful to know where the robot stopped."""
    error: Exception | None = None
    guard_name: str | None = None


class PolicyExecutor:
    """Runs one policy episode: observe → query policy → send actions → repeat.

    The policy is a pure function: obs → actions. It never signals "done".
    Execution runs until timeout_s expires or stop() is called externally.

    Images from WebRTC cameras can be included by passing a CameraSet.
    The executor waits for all cameras to produce frames before starting motion.
    """

    def __init__(
        self,
        feature_map: FeatureMap,
        policy: _PolicyFn | PolicyClient,
        *,
        cameras: CameraSet | None = None,
        config: PolicyRunnerConfig | None = None,
        safety_guards: list[SafetyGuard] | None = None,
        timeout_s: float = 0,
        inference_hz: float = 30,
    ) -> None:
        self._motion_groups = feature_map.get_motion_groups()
        self._feature_map = feature_map

        # Accept bare async function as policy (no wrapper needed)
        if callable(policy) and not hasattr(policy, "get_actions"):
            from policy.policy_client import CallbackPolicyClient  # noqa: PLC0415

            self._policy: PolicyClient = CallbackPolicyClient(policy)
        else:
            self._policy = policy  # type: ignore[assignment]

        self._cameras = cameras
        self._config = config
        self._safety_guards = safety_guards or []
        self._timeout_s = timeout_s
        self._inference_hz = inference_hz

        self._runner: PolicyRunner | None = None
        self._stop_event = asyncio.Event()
        self._last_obs: dict[str, Any] | None = None
        self._estop_error: EmergencyStopError | None = None
        self._estop_task: asyncio.Task[None] | None = None

        self.status = ExecutorStatus()
        self.result: ExecutionResult | None = None

    @property
    def phase(self) -> Phase:
        return self.status.phase

    @property
    def mg_ids(self) -> list[str]:
        return [mg.id for mg in self._motion_groups]

    @property
    def last_observation(self) -> dict[str, Any] | None:
        """The most recent observation (robot state + camera images).

        Available during and after execution. Returns None before the first
        observation is collected. This allows external code (UI, logging,
        dashboards) to inspect the current robot state while the executor runs.
        """
        return self._last_obs

    async def run(self) -> ExecutionResult:
        """Run execution, blocking until timeout or stop.

        Returns:
            ExecutionResult on normal termination (timeout, stopped).

        Raises:
            GuardStopError: A safety guard triggered.
            MotionError: Joint limit or self-collision detected.
            EmergencyStopError: E-stop or protective stop.
            RuntimeError: Connection lost or other error.
        """
        self._stop_event.clear()
        self.result = None
        await self._run()
        result = await self._cleanup()

        # Raise on exceptional stops so users handle them with try/except
        if result.reason == "safety_guard":
            raise GuardStopError(
                motion_group_id="",
                guard_name=result.guard_name or "unknown",
            )
        if result.reason == "motion_error":
            if isinstance(result.error, MotionError):
                raise result.error
            raise MotionError(motion_group_id="unknown", message=str(result.error))
        if result.reason == "estop":
            if isinstance(result.error, EmergencyStopError):
                raise result.error
            raise EmergencyStopError(controller_id="unknown")
        if result.reason in ("connection_lost", "error"):
            raise result.error if result.error else RuntimeError(result.reason)

        return result

    def stop(self) -> None:
        """Signal the executor to stop. Non-blocking — run() will return shortly after."""
        self._stop_event.set()

    async def _cleanup(self) -> ExecutionResult:
        """Clean up runner and policy connection, return final result."""
        if self._runner is not None:
            with contextlib.suppress(GuardStopError, MotionError, EmergencyStopError, OSError):
                await self._runner.stop()
            self._runner = None

        with contextlib.suppress(OSError, RuntimeError):
            await self._policy.close()

        self.status = ExecutorStatus(phase=Phase.IDLE)

        if self.result is None:
            self.result = ExecutionResult(reason="stopped", steps=self.status.step)
        logger.info(
            "PolicyExecutor stopped: reason=%s steps=%d duration=%.1fs",
            self.result.reason,
            self.result.steps,
            self.result.duration_s,
        )
        return self.result

    # -------------------------------------------------------------------------
    # Execution loop
    # -------------------------------------------------------------------------

    async def _run(self) -> None:
        """Main execution: open jogging, loop observe→act, close."""
        try:
            self._runner = PolicyRunner(
                motion_groups=self._motion_groups,
                config=self._config,
                tcp=self._feature_map.tcp,
                safety_guards=self._safety_guards,
            )

            # Connect cameras BEFORE opening jogging (ICE negotiation can take seconds)
            if self._cameras is not None:
                logger.info("Connecting cameras...")
                await self._cameras.connect()
                logger.info("All cameras ready")

            async with self._runner:
                # Start IO streams for FeatureMap (O(1) reads instead of HTTP)
                await self._feature_map.start()
                # Share IO cache values with sessions so guards can read IOs
                for cache in self._feature_map._io_caches:
                    self._runner.set_io_values_ref(cache._mg.id, cache.values)

                try:
                    await self._policy.connect(self.mg_ids)
                    self._estop_task = asyncio.create_task(
                        self._monitor_estop(), name="estop-monitor"
                    )
                    self.result = await self._execute()
                finally:
                    if self._estop_task is not None:
                        self._estop_task.cancel()
                        with contextlib.suppress(asyncio.CancelledError):
                            await self._estop_task
                        self._estop_task = None
                    if self._cameras is not None:
                        await self._cameras.disconnect()
                    await self._feature_map.stop()

        except asyncio.CancelledError:
            pass
        except (OSError, RuntimeError) as e:
            logger.error("Executor error: %s", e)
            self.status.message = f"Error: {e}"
            self.result = ExecutionResult(reason="error", steps=self.status.step, error=e)
        except Exception as e:
            logger.exception("Executor crashed")
            self.status.message = f"Error: {e}"
            self.result = ExecutionResult(reason="error", steps=self.status.step, error=e)
        finally:
            self.status.phase = Phase.IDLE

    async def _execute(self) -> ExecutionResult:
        """Run the observe-act loop until termination."""
        step = 0
        start_time = time.monotonic()
        interval = 1.0 / self._inference_hz
        last_obs: dict[str, Any] | None = None

        self.status.phase = Phase.EXECUTING
        self.status.message = "Running policy..."

        try:
            while not self._stop_event.is_set():
                # Check timeout
                if self._timeout_s > 0 and (time.monotonic() - start_time) >= self._timeout_s:
                    return self._result("timeout", step, start_time, last_obs)

                # Observe
                obs = await self._get_observation()
                if obs is None:
                    await asyncio.sleep(interval)
                    continue
                last_obs = obs
                self._last_obs = obs

                # Query policy
                action = await self._get_action(obs)

                # Send to robot
                await self._runner.send(action)
                step += 1
                self.status.step = step

                # Safety checks
                failure = self._check_session_failures(step, start_time, last_obs)
                if failure is not None:
                    return failure

                if self._estop_error is not None:
                    return ExecutionResult(
                        reason="estop",
                        steps=step,
                        duration_s=time.monotonic() - start_time,
                        last_state=last_obs,
                        error=self._estop_error,
                    )

                await asyncio.sleep(interval)

        except GuardStopError as e:
            return ExecutionResult(
                reason="safety_guard",
                steps=step,
                duration_s=time.monotonic() - start_time,
                last_state=last_obs,
                guard_name=e.guard_name,
            )
        except asyncio.CancelledError:
            pass
        except (OSError, RuntimeError) as e:
            return ExecutionResult(
                reason="error", steps=step, duration_s=time.monotonic() - start_time,
                last_state=last_obs, error=e,
            )

        return self._result("stopped", step, start_time, last_obs)

    # -------------------------------------------------------------------------
    # Observation + action helpers
    # -------------------------------------------------------------------------

    async def _get_observation(self) -> dict[str, Any] | None:
        robot_states = await self._runner.observe()
        if not robot_states:
            return None
        obs = await self._feature_map.build_observation(robot_states)

        # Add camera images to observation
        if self._cameras is not None:
            images = self._cameras.read()
            for cam_name, frame in images.items():
                obs[cam_name] = frame

        return obs

    async def _get_action(self, obs: dict[str, Any]) -> ActionChunk:
        """Query policy and ensure we get an ActionChunk back."""
        result = await self._policy.get_actions(obs)

        if isinstance(result, ActionChunk):
            return result

        # FeatureMap mode: policy returned a flat feature dict
        if isinstance(result, dict):
            joints, ios = self._feature_map.parse_action(result)
            if joints:
                return ActionChunk(joints=joints, ios=ios)

        msg = f"Policy must return ActionChunk or feature dict, got {type(result).__name__}"
        raise TypeError(msg)

    # -------------------------------------------------------------------------
    # Safety checks
    # -------------------------------------------------------------------------

    def _check_session_failures(
        self, step: int, start_time: float, last_obs: dict[str, Any] | None,
    ) -> ExecutionResult | None:
        for session in self._runner._sessions.values():
            if session.has_failed:
                reason_str = session.failure_reason or ""
                if "Safety guard" in reason_str:
                    guard_name = reason_str.split("'")[1] if "'" in reason_str else "unknown"
                    return ExecutionResult(
                        reason="safety_guard",
                        steps=step,
                        duration_s=time.monotonic() - start_time,
                        last_state=last_obs,
                        guard_name=guard_name,
                    )
                if "Motion error" in reason_str or "Jogging paused" in reason_str:
                    # Extract the original message (after "Motion error on 'mg_id': ")
                    msg = reason_str.split(": ", 1)[-1] if ": " in reason_str else reason_str
                    return ExecutionResult(
                        reason="motion_error",
                        steps=step,
                        duration_s=time.monotonic() - start_time,
                        last_state=last_obs,
                        error=MotionError(session.motion_group_id, msg),
                    )
                return ExecutionResult(
                    reason="connection_lost",
                    steps=step,
                    duration_s=time.monotonic() - start_time,
                    last_state=last_obs,
                    error=RuntimeError(session.failure_reason),
                )
        return None

    _OPERATIONAL_SAFETY_STATES = frozenset({"SAFETY_STATE_NORMAL", "SAFETY_STATE_REDUCED"})

    async def _monitor_estop(self) -> None:
        """Background task: stream controller state and detect e-stop.

        Uses the controller state-stream WebSocket (no polling).
        Sets ``_estop_error`` when any controller enters a non-operational
        safety state (e-stop, protective stop, fault, etc.).
        """
        # Deduplicate controllers (multiple motion groups may share one)
        seen: set[str] = set()
        controllers: list[tuple[str, object]] = []
        for mg in self._motion_groups:
            if mg._controller_id not in seen:
                seen.add(mg._controller_id)
                controllers.append((mg._controller_id, mg._api_client))

        tasks = [asyncio.create_task(self._watch_controller(cid, api)) for cid, api in controllers]
        try:
            await asyncio.gather(*tasks)
        except asyncio.CancelledError:
            for t in tasks:
                t.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)

    async def _watch_controller(self, controller_id: str, api_client: object) -> None:
        """Watch a single controller's state stream for safety state changes."""
        cell = self._motion_groups[0]._cell
        stream = None
        try:
            stream = api_client.controller_api.stream_robot_controller_state(
                cell=cell,
                controller=controller_id,
                response_rate=1000,
            )
            async for state in stream:
                safety_raw = getattr(state, "safety_state", None)
                if safety_raw is None:
                    continue
                safety = safety_raw.name if hasattr(safety_raw, "name") else str(safety_raw)
                if safety not in self._OPERATIONAL_SAFETY_STATES:
                    logger.error("E-stop detected on %s: %s", controller_id, safety)
                    self._estop_error = EmergencyStopError(controller_id, safety)
                    return
        except asyncio.CancelledError:
            raise
        except (OSError, RuntimeError):
            pass
        finally:
            if stream is not None:
                with contextlib.suppress(Exception):
                    await stream.aclose()

    @staticmethod
    def _result(
        reason: str, step: int, start_time: float, last_obs: dict[str, Any] | None = None,
    ) -> ExecutionResult:
        return ExecutionResult(
            reason=reason, steps=step, duration_s=time.monotonic() - start_time, last_state=last_obs,
        )
