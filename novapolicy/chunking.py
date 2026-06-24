"""Pure action-chunk transforms used by :class:`~novapolicy.executor.PolicyExecutor`.

These are deliberately free functions with no executor state: they take a
chunk (and whatever context they need) and return a new chunk or a scalar.
Keeping them here keeps the executor focused on orchestration and makes the
transforms trivially unit-testable.
"""

from __future__ import annotations

from dataclasses import dataclass
import logging
from typing import TYPE_CHECKING, Any

from novapolicy.types import ActionChunk

if TYPE_CHECKING:
    from collections.abc import Iterable

NOW = -1
"""Anchor sentinel: resolve "now" at yield time (see :data:`novapolicy.jogging.waypoints.NOW`)."""

logger = logging.getLogger(__name__)


def _max_steps(chunk: ActionChunk) -> int:
    """Longest step sequence across all joint and TCP motion groups."""
    return max(
        (len(steps) for steps in (*chunk.joints.values(), *chunk.tcp.values())),
        default=0,
    )


def chunk_duration_s(chunk: ActionChunk) -> float:
    """Compute how long a chunk takes to execute (seconds)."""
    if chunk.dt_ms <= 0:
        return 0.0
    return _max_steps(chunk) * chunk.dt_ms / 1000.0


def trim_chunk(chunk: ActionChunk, n: int) -> ActionChunk:
    """Trim an action chunk to its first ``n`` steps (receding horizon).

    Returns the chunk unmodified if ``n <= 0`` (execute all) or if it is
    already shorter than ``n``. Warns if the policy returned fewer steps than
    requested.
    """
    if n <= 0:
        return chunk

    actual_steps = _max_steps(chunk)
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
        first_timestamp_ms=chunk.first_timestamp_ms,
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
        first_timestamp_ms=chunk.first_timestamp_ms,
        seam_backdate_steps=chunk.seam_backdate_steps,
    )


@dataclass(frozen=True, slots=True)
class Placement:
    """Where a chunk's first step sits on one session's server timeline.

    ``anchor_ms`` is an explicit absolute anchor, or :data:`NOW` to resolve
    "now" at yield time. ``anchor_offset_steps`` shifts that anchor by whole
    ``dt`` steps: ``+1`` = one step ahead (sequential), negative = backdated
    (RTC seam), ``0`` = exact.
    """

    anchor_ms: int
    anchor_offset_steps: int


def placement(chunk: ActionChunk, *, policy_rate_hz: float) -> Placement:
    """Decide how a chunk is anchored on one session's timeline.

    The "now" component is intentionally NOT resolved here — it is computed at
    yield time in :func:`novapolicy.jogging.waypoints.make_waypoints_request` so the
    anchor cannot go stale while the chunk waits in the session queue. This
    matters most for RTC, whose seam alignment is sensitive to that delay.

    * An explicit ``chunk.first_timestamp_ms`` (>=0) set by the policy wins —
      used verbatim, anchored exactly.
    * Wait-for-chunk (``policy_rate_hz < 0``) — "now", one step ahead.
    * Otherwise (typically RTC) — "now", backdated by ``seam_backdate_steps`` so
      the step matching the robot's current position lands at "now".
    """
    if chunk.first_timestamp_ms >= 0:
        return Placement(chunk.first_timestamp_ms, 0)
    if policy_rate_hz < 0:
        return Placement(NOW, 1)
    return Placement(NOW, -chunk.seam_backdate_steps)
