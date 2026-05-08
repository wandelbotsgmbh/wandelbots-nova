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

from policy._sdk import get_controller_id
from policy.estop import EstopMonitor, check_estop, check_sessions
from policy.io import IOStreamCache
from policy.runner import PolicyRunner
from policy.types import ActionChunk, EmergencyStopError, GuardStopError, MotionError

if TYPE_CHECKING:
    from policy.cameras import CameraSource
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
    """

    def __init__(
        self,
        feature_map: FeatureMap,
        policy: _PolicyFn | PolicyClient,
        *,
        cameras: CameraSource | None = None,
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
        self._estop_monitor: EstopMonitor | None = None
        self._io_caches: list[IOStreamCache] = []

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
        observation is collected.
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
    # Execution lifecycle
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

            if self._cameras is not None:
                logger.info("Connecting cameras...")
                await self._cameras.connect()
                logger.info("All cameras ready")

            async with self._runner:
                await self._start_io_streams()

                try:
                    await self._policy.connect(self.mg_ids)
                    self._estop_monitor = EstopMonitor(self._motion_groups)
                    await self._estop_monitor.start()
                    self.result = await self._execute()
                finally:
                    if self._estop_monitor is not None:
                        await self._estop_monitor.stop()
                        self._estop_monitor = None
                    if self._cameras is not None:
                        await self._cameras.disconnect()
                    await self._stop_io_streams()

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

    # -------------------------------------------------------------------------
    # Observe → act loop
    # -------------------------------------------------------------------------

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
                if self._timeout_s > 0 and (time.monotonic() - start_time) >= self._timeout_s:
                    return _result("timeout", step, start_time, last_obs)

                # Observe
                robot_states = await self._runner.observe()
                if not robot_states:
                    await asyncio.sleep(interval)
                    continue
                images = self._cameras.read() if self._cameras else None
                self._last_obs = robot_states
                last_obs = robot_states

                # Query policy → send to robot
                action = await self._policy.get_actions(
                    robot_states, self._feature_map, images, self._all_io_values or None,
                )
                await self._runner.send(action)
                step += 1
                self.status.step = step

                # Check failures
                check_sessions(self._runner._sessions)
                check_estop(self._estop_monitor)

                await asyncio.sleep(interval)

        except GuardStopError as e:
            return ExecutionResult(
                reason="safety_guard",
                steps=step,
                duration_s=time.monotonic() - start_time,
                last_state=last_obs,
                guard_name=e.guard_name,
            )
        except MotionError as e:
            return ExecutionResult(
                reason="motion_error", steps=step,
                duration_s=time.monotonic() - start_time,
                last_state=last_obs, error=e,
            )
        except EmergencyStopError as e:
            return ExecutionResult(
                reason="estop", steps=step,
                duration_s=time.monotonic() - start_time,
                last_state=last_obs, error=e,
            )
        except asyncio.CancelledError:
            pass
        except (OSError, RuntimeError) as e:
            return ExecutionResult(
                reason="error", steps=step, duration_s=time.monotonic() - start_time,
                last_state=last_obs, error=e,
            )

        return _result("stopped", step, start_time, last_obs)

    # -------------------------------------------------------------------------
    # IO stream management
    # -------------------------------------------------------------------------

    async def _start_io_streams(self) -> None:
        """Open IO WebSocket streams and wire caches to sessions for guards."""
        io_by_ctrl = self._feature_map.io_keys_by_controller()
        for group in self._feature_map.groups:
            ctrl_id = get_controller_id(group.motion_group)
            io_keys = io_by_ctrl.get(ctrl_id)
            if not io_keys:
                continue
            if any(get_controller_id(c.motion_group) == ctrl_id for c in self._io_caches):
                continue
            cache = IOStreamCache(group.motion_group, io_keys)
            self._io_caches.append(cache)
            await cache.start()

        for cache in self._io_caches:
            self._runner.set_io_values_ref(cache.motion_group.id, cache.values)

    async def _stop_io_streams(self) -> None:
        """Close all IO streams."""
        for cache in self._io_caches:
            await cache.stop()
        self._io_caches.clear()

    @property
    def _all_io_values(self) -> dict[str, object]:
        """Merged IO values from all caches."""
        merged: dict[str, object] = {}
        for cache in self._io_caches:
            merged.update(cache.values)
        return merged


def _result(
    reason: str, step: int, start_time: float, last_obs: dict[str, Any] | None = None,
) -> ExecutionResult:
    return ExecutionResult(
        reason=reason, steps=step, duration_s=time.monotonic() - start_time, last_state=last_obs,
    )

