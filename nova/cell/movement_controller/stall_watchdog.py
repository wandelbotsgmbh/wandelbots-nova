"""Shared completion watchdog for movement controllers.

Every movement controller (``move_forward`` and — as a follow-up — the
``TrajectoryCursor``) awaits the controller's terminal completion event on the
motion-group state stream. If that event is lost or delayed while the robot is
physically at rest, the await blocks forever ("waiting for completion or error"
with no completion). This module provides a reusable observation object plus a
watchdog coroutine that resolves such stalls instead of hanging.

The watchdog never fakes completion (at-target is verified against the planned
final joints) and never cancels in-flight motion (it only acts once the robot
is already at standstill).

Follow-up: ``TrajectoryCursor`` (``trajectory_cursor.py``) shares the same
``TrajectoryExecutionMachine`` and therefore the same latent hang. It has a
different, multi-operation completion model (per-operation futures), so wiring
this watchdog in is tracked separately rather than rushed here.
"""

from __future__ import annotations

import asyncio
import logging

from nova import api
from nova.exceptions import MovementStalled

logger = logging.getLogger(__name__)


def joints_of(state: api.models.MotionGroupState) -> tuple[float, ...]:
    jp = state.joint_position
    return tuple(getattr(jp, "root", jp))


def is_at_target(
    joints: tuple[float, ...] | None, target: tuple[float, ...] | None, tolerance: float
) -> bool:
    """True when every joint is within ``tolerance`` of the planned target.
    Units follow the joint (rad for revolute, mm for prismatic). ``None`` target
    or a dimension mismatch returns False (cannot confirm)."""
    if target is None or joints is None or len(target) != len(joints):
        return False
    return all(abs(a - b) <= tolerance for a, b in zip(joints, target))


def is_legitimate_wait(state: api.models.MotionGroupState) -> bool:
    """True when the robot is at rest *by design* (waiting on IO or paused), so
    the watchdog must not treat the standstill as a stall."""
    if state.execute is None or not isinstance(state.execute.details, api.models.TrajectoryDetails):
        return False
    return isinstance(
        state.execute.details.state,
        (
            api.models.TrajectoryWaitForIO,
            api.models.TrajectoryPausedOnIO,
            api.models.TrajectoryPausedByUser,
        ),
    )


class StandstillObservation:
    """Latest standstill/position facts, written by a controller's state monitor
    and read by :func:`run_standstill_watchdog`. Plain object (no locking: single
    event loop, cooperative scheduling)."""

    def __init__(self) -> None:
        self.standstill: bool = False
        self.standstill_since: float | None = None  # event-loop time
        self.in_legitimate_wait: bool = False
        self.joints: tuple[float, ...] | None = None

    def update(self, state: api.models.MotionGroupState, now: float) -> None:
        self.joints = joints_of(state) or self.joints
        self.in_legitimate_wait = is_legitimate_wait(state)
        if state.standstill:
            if not self.standstill:
                self.standstill_since = now
            self.standstill = True
        else:
            self.standstill = False
            self.standstill_since = None


async def run_standstill_watchdog(
    obs: StandstillObservation,
    *,
    motion_id: str,
    target_joint_position: tuple[float, ...] | None,
    stall_timeout_s: float,
    max_stall_s: float,
    at_target_tolerance: float,
) -> None:
    """Resolve a lost / delayed terminal completion event.

    Backstop for the cases the state monitor's standstill-after-end handling
    (Change A) does NOT cover — i.e. where ``TrajectoryEnded`` was never observed
    at all, so the machine is stuck in ``executing`` rather than ``ending``.
    When the robot sits at an unexplained standstill (not waiting on IO, not
    paused), resolve the move instead of blocking forever:

    * **At target** (after ``stall_timeout_s``) -> return (complete). Covers a
      never-delivered ``TrajectoryEnded`` and zero-motion moves already at target.
    * **Still at standstill past ``max_stall_s``** -> raise (hard ceiling).
      Catches the ambiguous stop-short with no ``TrajectoryEnded`` (e.g. a real
      protective stop), which is otherwise indistinguishable from a legitimate
      in-path dwell — so execute() can never hang forever.

    Returns on a confirmed completion; raises :class:`MovementStalled` otherwise.
    """
    loop = asyncio.get_running_loop()
    poll = min(0.1, max(stall_timeout_s / 4, 0.01))
    while True:
        await asyncio.sleep(poll)
        if obs.in_legitimate_wait or not obs.standstill or obs.standstill_since is None:
            continue
        held = loop.time() - obs.standstill_since
        if held >= stall_timeout_s and is_at_target(
            obs.joints, target_joint_position, at_target_tolerance
        ):
            return
        if held >= max_stall_s:
            raise MovementStalled(
                f"Trajectory {motion_id} stalled: robot at standstill for >= {max_stall_s}s "
                f"with no completion event and not at target "
                f"(joints={obs.joints}, target={target_joint_position})"
            )
