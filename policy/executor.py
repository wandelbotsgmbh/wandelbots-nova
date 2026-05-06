"""PolicyExecutor — runs one policy episode via PID-controlled jogging."""

from __future__ import annotations

import asyncio
import contextlib
import logging
import time
from dataclasses import dataclass
from enum import StrEnum
from typing import TYPE_CHECKING, Any

from policy.runner import PolicyRunner
from policy.types import ActionChunk, EmergencyStopError, GuardStopError, MotionError

if TYPE_CHECKING:
    from nova.cell.motion_group import MotionGroup
    from policy.cameras import CameraSet
    from policy.feature_map import FeatureMap
    from policy.policy_client import PolicyClient
    from policy.types import PolicyRunnerConfig, SafetyGuard

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

    Two observation modes:
    1. Direct: pass motion_groups, policy receives {mg_id: RobotState}.
    2. FeatureMap: pass a FeatureMap, policy receives flat feature dicts.

    Images from WebRTC cameras can be included by passing a CameraSet.
    The executor waits for all cameras to produce frames before starting motion.
    """

    def __init__(
        self,
        motion_groups: list[MotionGroup] | None = None,
        policy: PolicyClient | None = None,
        *,
        feature_map: FeatureMap | None = None,
        cameras: CameraSet | None = None,
        config: PolicyRunnerConfig | None = None,
        safety_guards: list[SafetyGuard] | None = None,
        timeout_s: float = 0,
        rate_hz: float = 30,
    ) -> None:
        if feature_map is not None:
            self._motion_groups = feature_map.get_motion_groups()
            self._feature_map = feature_map
        elif motion_groups is not None:
            self._motion_groups = motion_groups
            self._feature_map = None
        else:
            msg = "Provide either motion_groups or feature_map"
            raise ValueError(msg)

        if policy is None:
            msg = "policy is required"
            raise ValueError(msg)

        self._policy = policy
        self._cameras = cameras
        self._config = config
        self._safety_guards = safety_guards or []
        self._timeout_s = timeout_s
        self._rate_hz = rate_hz

        self._runner: PolicyRunner | None = None
        self._stop_event = asyncio.Event()
        self._last_obs: dict[str, Any] | None = None

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
                safety_guards=self._safety_guards,
            )

            # Connect cameras BEFORE opening jogging (ICE negotiation can take seconds)
            if self._cameras is not None:
                logger.info("Connecting cameras...")
                await self._cameras.connect()
                logger.info("All cameras ready")

            async with self._runner:
                # Start IO streams for FeatureMap (O(1) reads instead of HTTP)
                if self._feature_map is not None:
                    await self._feature_map.start()
                    # Share IO cache values with sessions so guards can read IOs
                    for cache in self._feature_map._io_caches:
                        self._runner.set_io_values_ref(cache._mg.id, cache.values)

                try:
                    await self._policy.connect(self.mg_ids)
                    self.result = await self._execute()
                finally:
                    if self._cameras is not None:
                        await self._cameras.disconnect()
                    if self._feature_map is not None:
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
        interval = 1.0 / self._rate_hz
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

                estop = await self._check_estop(step, start_time, last_obs)
                if estop is not None:
                    return estop

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
        if self._feature_map is not None:
            obs = await self._feature_map.build_observation(robot_states)
        else:
            obs = robot_states

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
        if isinstance(result, dict) and self._feature_map is not None:
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

    async def _check_estop(
        self, step: int, start_time: float, last_obs: dict[str, Any] | None,
    ) -> ExecutionResult | None:
        """Check e-stop every ~1 second (every 30 steps at 30Hz)."""
        if step % 30 != 0:
            return None
        for mg in self._motion_groups:
            result = await self._check_estop_for_group(mg, step, start_time, last_obs)
            if result is not None:
                return result
        return None

    async def _check_estop_for_group(
        self, mg: MotionGroup, step: int, start_time: float, last_obs: dict[str, Any] | None,
    ) -> ExecutionResult | None:
        try:
            estop_state = await mg._api_client.virtual_controller_api.get_emergency_stop(
                cell=mg._cell, controller=mg._controller_id
            )
            if estop_state.active:
                return self._safety_stop_result(
                    mg._controller_id, "SAFETY_STATE_DEVICE_EMERGENCY_STOP",
                    step, start_time, last_obs,
                )
        except (OSError, RuntimeError, ValueError):
            return await self._check_safety_state(mg, step, start_time, last_obs)
        return None

    _OPERATIONAL_SAFETY_STATES = frozenset({"SAFETY_STATE_NORMAL", "SAFETY_STATE_REDUCED"})

    async def _check_safety_state(
        self, mg: MotionGroup, step: int, start_time: float, last_obs: dict[str, Any] | None,
    ) -> ExecutionResult | None:
        try:
            ctrl_state = await mg._api_client.controller_api.get_current_robot_controller_state(
                cell=mg._cell, controller=mg._controller_id
            )
            safety = str(getattr(ctrl_state, "safety_state", ""))
            if safety and safety not in self._OPERATIONAL_SAFETY_STATES:
                return self._safety_stop_result(
                    mg._controller_id, safety, step, start_time, last_obs,
                )
        except (OSError, RuntimeError, ValueError):
            pass
        return None

    def _safety_stop_result(
        self,
        controller_id: str,
        safety_state: str,
        step: int,
        start_time: float,
        last_obs: dict[str, Any] | None,
    ) -> ExecutionResult:
        logger.error("Safety stop on %s: %s", controller_id, safety_state)
        return ExecutionResult(
            reason="estop",
            steps=step,
            duration_s=time.monotonic() - start_time,
            last_state=last_obs,
            error=EmergencyStopError(controller_id, safety_state),
        )

    @staticmethod
    def _result(
        reason: str, step: int, start_time: float, last_obs: dict[str, Any] | None = None,
    ) -> ExecutionResult:
        return ExecutionResult(
            reason=reason, steps=step, duration_s=time.monotonic() - start_time, last_state=last_obs,
        )
