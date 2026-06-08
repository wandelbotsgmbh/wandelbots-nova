"""Pure action-chunk transforms used by :class:`~policy.executor.PolicyExecutor`.

These are deliberately free functions with no executor state: they take a
chunk (and whatever context they need) and return a new chunk or a scalar.
Keeping them here keeps the executor focused on orchestration and makes the
transforms trivially unit-testable.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Any

from policy.types import ActionChunk

if TYPE_CHECKING:
    from collections.abc import Iterable

logger = logging.getLogger(__name__)


def chunk_duration_s(chunk: ActionChunk) -> float:
    """Compute how long a chunk takes to execute (seconds)."""
    if chunk.dt_ms <= 0:
        return 0.0
    n_steps = 0
    for steps in chunk.joints.values():
        n_steps = max(n_steps, len(steps))
    for steps in chunk.tcp.values():
        n_steps = max(n_steps, len(steps))
    return n_steps * chunk.dt_ms / 1000.0


def trim_chunk(chunk: ActionChunk, n: int) -> ActionChunk:
    """Trim an action chunk to its first ``n`` steps (receding horizon).

    Returns the chunk unmodified if ``n <= 0`` (execute all) or if it is
    already shorter than ``n``. Warns if the policy returned fewer steps than
    requested.
    """
    if n <= 0:
        return chunk

    actual_steps = max(
        (len(steps) for steps in chunk.joints.values()),
        default=max((len(steps) for steps in chunk.tcp.values()), default=0),
    )
    if 0 < actual_steps < n:
        logger.warning(
            "Policy returned %d steps but n_action_steps=%d. "
            "Check that n_action_steps <= policy action_horizon.",
            actual_steps,
            n,
        )

    trimmed_joints = (
        {mg_id: steps[:n] for mg_id, steps in chunk.joints.items()} if chunk.joints else {}
    )
    trimmed_tcp = {mg_id: steps[:n] for mg_id, steps in chunk.tcp.items()} if chunk.tcp else {}

    return ActionChunk(
        joints=trimmed_joints,
        tcp=trimmed_tcp,
        ios=chunk.ios,
        dt_ms=chunk.dt_ms,
        start_time_ms=chunk.start_time_ms,
        seam_backdate_steps=chunk.seam_backdate_steps,
    )


def apply_relative_mode(
    chunk: ActionChunk,
    states: dict[str, Any],
    relative_mgs: Iterable[str],
) -> ActionChunk:
    """Convert relative (delta) action targets to absolute positions.

    For motion groups in ``relative_mgs``, each step is an offset from the
    robot's state at inference time; joint deltas accumulate across steps.
    Returns the chunk unmodified when ``relative_mgs`` is empty.
    """
    relative_mgs = list(relative_mgs)
    if not relative_mgs:
        return chunk

    new_joints = dict(chunk.joints)
    new_tcp = dict(chunk.tcp)

    for mg_id in relative_mgs:
        state = states.get(mg_id)
        if state is None:
            continue

        # Relative joint actions: each step is a delta from the previous,
        # so step[i] target = current + sum(deltas[0..i]).
        if mg_id in new_joints:
            running = list(state.joints)
            abs_steps = []
            for step in new_joints[mg_id]:
                running = [r + d for r, d in zip(running, step, strict=True)]
                abs_steps.append(list(running))
            new_joints[mg_id] = abs_steps

        # Relative TCP actions.
        if mg_id in new_tcp and hasattr(state, "pose") and state.pose is not None:
            current_tcp = list(state.pose.position) + list(state.pose.orientation)
            new_tcp[mg_id] = [
                [c + d for c, d in zip(current_tcp, step, strict=True)] for step in new_tcp[mg_id]
            ]

    return ActionChunk(
        joints=new_joints,
        tcp=new_tcp,
        ios=chunk.ios,
        dt_ms=chunk.dt_ms,
        start_time_ms=chunk.start_time_ms,
        seam_backdate_steps=chunk.seam_backdate_steps,
    )


def placement_start_ms(
    chunk: ActionChunk,
    *,
    policy_rate_hz: float,
    session_elapsed_ms: int,
    backdate_ms: int,
) -> int:
    """Compute the ``start_time_ms`` for a chunk on one session.

    * An explicit ``chunk.start_time_ms`` (>=0) set by the policy always wins.
    * Wait-for-chunk (``policy_rate_hz < 0``) — sequential, non-overlapping:
      relative placement (``-1``), i.e. start from the robot's current position
      "now". There is no shared timeline to align to, so this avoids the slow
      drift you'd get from re-using an absolute anchor across many chunks.
    * Overlapping (``policy_rate_hz >= 0``, typically RTC) — absolute
      timestamps anchored at send time, backdated by ``backdate_ms`` so the
      step matching the robot's current position lands at "now". Relative
      placement cannot express this, hence the two models exist.
    """
    if chunk.start_time_ms >= 0:
        return chunk.start_time_ms
    if policy_rate_hz < 0:
        return -1
    return max(0, session_elapsed_ms - backdate_ms)
