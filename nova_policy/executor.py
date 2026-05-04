"""PolicyExecutor — high-level lifecycle manager for policy execution."""

from __future__ import annotations

import asyncio
import contextlib
import logging
import time
from collections.abc import Callable, Coroutine
from dataclasses import dataclass
from enum import StrEnum
from typing import TYPE_CHECKING, Any

from nova.cell.motion_group import MotionGroup
from nova.types import RobotState

from nova_policy.feature_map import FeatureMap
from nova_policy.runner import PolicyRunner
from nova_policy.types import ActionChunk, PolicyDone, PolicyWaiting, SafetyStopError

if TYPE_CHECKING:
    from nova_policy.policy_client import PolicyClient
    from nova_policy.types import PolicyRunnerConfig, SafetyGuard

logger = logging.getLogger(__name__)


class Phase(StrEnum):
    """Executor lifecycle phase."""

    IDLE = "IDLE"
    RESETTING = "RESETTING"
    READY = "READY"
    EXECUTING = "EXECUTING"


@dataclass
class ExecutorStatus:
    """Current executor state, queryable at any time."""

    phase: Phase = Phase.IDLE
    step: int = 0
    episode: int = 0
    message: str = ""


@dataclass
class EpisodeResult:
    """Result of a single policy episode."""

    reason: str
    """Why the episode ended: 'done' | 'stopped' | 'timeout' | 'safety_guard' | 'estop' | 'error'"""

    steps: int = 0
    duration_s: float = 0.0
    error: Exception | None = None
    guard_name: str | None = None


# Type for the reset callback
ResetCallback = Callable[[list[MotionGroup]], Coroutine[Any, Any, None]]

# Type for the observation builder callback
BuildObsCallback = Callable[
    [dict[str, RobotState], list[MotionGroup]], Coroutine[Any, Any, dict[str, Any]]
]

# Fatal reasons that stop the executor (no retry, no next episode)
_FATAL_REASONS = frozenset({"stopped", "error", "safety_guard", "connection_lost", "estop"})


class PolicyExecutor:
    """High-level lifecycle manager for policy execution.

    Manages the full cycle: reset → ready → execute → reset → ready → ...
    Stays alive across multiple policy episodes until explicitly stopped.

    Two modes:
    1. Direct: pass motion_groups, policy uses motion group IDs as keys.
    2. FeatureMap: pass a FeatureMap, policy uses flat LeRobot-style feature dicts.
    """

    def __init__(
        self,
        motion_groups: list[MotionGroup] | None = None,
        policy: PolicyClient | None = None,
        *,
        feature_map: FeatureMap | None = None,
        on_reset: ResetCallback | None = None,
        build_obs: BuildObsCallback | None = None,
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
        self._on_reset = on_reset
        self._build_obs = build_obs
        self._config = config
        self._safety_guards = safety_guards or []
        self._timeout_s = timeout_s
        self._rate_hz = rate_hz

        self._runner: PolicyRunner | None = None
        self._task: asyncio.Task[None] | None = None
        self._stop_event = asyncio.Event()

        self.status = ExecutorStatus()

    @property
    def phase(self) -> Phase:
        return self.status.phase

    @property
    def mg_list(self) -> list[MotionGroup]:
        return list(self._motion_groups)

    @property
    def mg_ids(self) -> list[str]:
        return [mg.id for mg in self._motion_groups]

    async def start(self) -> None:
        """Start the executor. Runs in background until stop() is called."""
        if self._task is not None and not self._task.done():
            logger.warning("Executor already running")
            return

        self._stop_event.clear()
        self._task = asyncio.create_task(self._loop(), name="policy-executor")
        while self.status.phase not in (Phase.READY, Phase.EXECUTING):
            await asyncio.sleep(0.05)
            if self._task.done():
                break

    async def stop(self, reason: str = "stopped") -> None:
        """Stop the executor. Cancels execution, closes connections."""
        self._stop_event.set()
        if self._task is not None:
            self._task.cancel()
            with contextlib.suppress(asyncio.CancelledError, SafetyStopError, OSError):
                await self._task
            self._task = None

        if self._runner is not None:
            with contextlib.suppress(SafetyStopError, OSError):
                await self._runner.stop()
            self._runner = None

        await self._notify_and_close(reason)
        self.status = ExecutorStatus(phase=Phase.IDLE)
        logger.info("PolicyExecutor stopped (reason=%s)", reason)

    # -------------------------------------------------------------------------
    # Main loop
    # -------------------------------------------------------------------------

    async def _loop(self) -> None:
        """Main executor loop: reset → ready → execute → repeat."""
        try:
            self._runner = PolicyRunner(
                motion_groups=self._motion_groups,
                config=self._config,
                safety_guards=self._safety_guards,
            )

            async with self._runner:
                await self._policy.connect(self.mg_ids)

                while not self._stop_event.is_set():
                    await self._do_reset()
                    result = await self._run_episode()
                    self.status.episode += 1
                    logger.info(
                        "Episode %d ended: reason=%s, steps=%d, duration=%.1fs",
                        self.status.episode,
                        result.reason,
                        result.steps,
                        result.duration_s,
                    )
                    if result.reason in _FATAL_REASONS:
                        await self._notify_and_close(result.reason)
                        break

        except asyncio.CancelledError:
            pass
        except (OSError, RuntimeError) as e:
            logger.error("Executor error: %s", e)
            self.status.message = f"Error: {e}"
        finally:
            self.status.phase = Phase.IDLE
            self.status.step = 0

    async def _notify_and_close(self, reason: str) -> None:
        """Best-effort: notify policy and close connection."""
        with contextlib.suppress(OSError, RuntimeError):
            await self._policy.notify_stopped(reason)
        with contextlib.suppress(OSError, RuntimeError):
            await self._policy.close()

    # -------------------------------------------------------------------------
    # Reset
    # -------------------------------------------------------------------------

    async def _do_reset(self) -> None:
        """Run the reset callback (move to home, open grippers, etc.)."""
        if self._on_reset is None:
            self.status.phase = Phase.READY
            self.status.message = "Waiting for policy..."
            return

        self.status.phase = Phase.RESETTING
        self.status.message = "Resetting robots..."
        logger.info("Resetting robots (episode %d)...", self.status.episode + 1)

        for session in self._runner._sessions.values():
            if session.is_running:
                await session.stop()

        await asyncio.sleep(0.5)
        await self._on_reset(self.mg_list)

        for session in self._runner._sessions.values():
            await session.start()

        self.status.phase = Phase.READY
        self.status.message = "Waiting for policy..."
        logger.info("Reset complete. READY.")

    # -------------------------------------------------------------------------
    # Episode execution (split into small methods for readability)
    # -------------------------------------------------------------------------

    async def _run_episode(self) -> EpisodeResult:
        """Run one policy episode: pull actions until done/timeout/stop."""
        step = 0
        start_time = time.monotonic()
        interval = 1.0 / self._rate_hz

        self.status.message = "Waiting for policy..."

        try:
            while not self._stop_event.is_set():
                if self._is_timed_out(start_time):
                    return self._result("timeout", step, start_time)

                tick_result = await self._process_tick(step, start_time)
                if isinstance(tick_result, EpisodeResult):
                    return tick_result
                if isinstance(tick_result, int):
                    step = tick_result

                await asyncio.sleep(interval)

        except SafetyStopError as e:
            return EpisodeResult(
                reason="safety_guard",
                steps=step,
                duration_s=time.monotonic() - start_time,
                guard_name=e.guard_name,
            )
        except asyncio.CancelledError:
            pass
        except (OSError, RuntimeError) as e:
            return EpisodeResult(
                reason="error", steps=step, duration_s=time.monotonic() - start_time, error=e
            )

        reason = "stopped" if self._stop_event.is_set() else "done"
        return self._result(reason, step, start_time)

    async def _process_tick(self, step: int, start_time: float) -> EpisodeResult | int | None:
        """Process one tick. Returns EpisodeResult to end, new step count, or None to skip."""
        obs = await self._get_observation()
        if obs is None:
            return None

        result = await self._policy.get_actions(obs)
        action = self._handle_policy_result(result)

        if action is None:
            if isinstance(result, PolicyDone) and step > 0:
                return self._result("done", step, start_time)
            return None

        if self.status.phase == Phase.READY:
            self._enter_executing()

        await self._runner.send(action)
        step += 1
        self.status.step = step

        failure = self._check_session_failures(step, start_time)
        if failure is not None:
            return failure

        estop = await self._check_estop(step, start_time)
        if estop is not None:
            return estop

        return step

    def _is_timed_out(self, start_time: float) -> bool:
        if self._timeout_s <= 0:
            return False
        return (time.monotonic() - start_time) >= self._timeout_s

    async def _get_observation(self) -> dict[str, Any] | None:
        robot_states = await self._runner.observe()
        if not robot_states:
            return None
        if self._feature_map is not None:
            return await self._feature_map.build_observation(robot_states)
        if self._build_obs is not None:
            return await self._build_obs(robot_states, self.mg_list)
        return robot_states

    def _handle_policy_result(self, result: object) -> ActionChunk | None:
        """Extract ActionChunk from policy result, or None if not actionable."""
        if isinstance(result, (PolicyDone, PolicyWaiting)):
            return None
        if isinstance(result, ActionChunk):
            return result
        # Feature map mode: policy client may return a raw flat dict
        # (when using a custom client that doesn't wrap in ActionChunk)
        if isinstance(result, dict) and self._feature_map is not None:
            joints, ios = self._feature_map.parse_action(result)
            if joints:
                return ActionChunk(joints=joints, ios=ios)
        return None

    def _enter_executing(self) -> None:
        self.status.phase = Phase.EXECUTING
        self.status.message = "Running policy..."
        logger.info("Policy started outputting actions")

    def _check_session_failures(self, step: int, start_time: float) -> EpisodeResult | None:
        for session in self._runner._sessions.values():
            if session.has_failed:
                logger.error(
                    "Jogging session failed for %s: %s",
                    session.motion_group_id,
                    session.failure_reason,
                )
                return EpisodeResult(
                    reason="connection_lost",
                    steps=step,
                    duration_s=time.monotonic() - start_time,
                    error=RuntimeError(session.failure_reason),
                )
        return None

    async def _check_estop(self, step: int, start_time: float) -> EpisodeResult | None:
        """Check e-stop every ~1 second (every 30 steps)."""
        if step % 30 != 0:
            return None
        for mg in self.mg_list:
            result = await self._check_estop_for_group(mg, step, start_time)
            if result is not None:
                return result
        return None

    async def _check_estop_for_group(
        self, mg: MotionGroup, step: int, start_time: float
    ) -> EpisodeResult | None:
        """Check e-stop for a single motion group."""
        try:
            estop_state = await mg._api_client.virtual_controller_api.get_emergency_stop(
                cell=mg._cell, controller=mg._controller_id
            )
            if estop_state.active:
                return self._estop_result(mg._controller_id, step, start_time)
        except (OSError, RuntimeError, ValueError):
            # Not a virtual controller — check safety state
            return await self._check_safety_state(mg, step, start_time)
        return None

    async def _check_safety_state(
        self, mg: MotionGroup, step: int, start_time: float
    ) -> EpisodeResult | None:
        """Check safety state for real robot controllers."""
        try:
            ctrl_state = await mg._api_client.controller_api.get_current_robot_controller_state(
                cell=mg._cell, controller=mg._controller_id
            )
            safety = str(getattr(ctrl_state, "safety_state", "")).upper()
            if "ESTOP" in safety or "EMERGENCY" in safety:
                return self._estop_result(mg._controller_id, step, start_time)
        except (OSError, RuntimeError, ValueError):
            pass
        return None

    def _estop_result(self, controller_id: str, step: int, start_time: float) -> EpisodeResult:
        logger.error("E-stop detected on %s", controller_id)
        return EpisodeResult(
            reason="estop",
            steps=step,
            duration_s=time.monotonic() - start_time,
            error=RuntimeError(f"E-stop on {controller_id}"),
        )

    @staticmethod
    def _result(reason: str, step: int, start_time: float) -> EpisodeResult:
        return EpisodeResult(reason=reason, steps=step, duration_s=time.monotonic() - start_time)
