"""Pure action-chunk transforms used by :class:`~novapolicy.executor.PolicyExecutor`.

These are deliberately free functions with no executor state: they take a
chunk (and whatever context they need) and return a new chunk or a scalar.
Keeping them here keeps the executor focused on orchestration and makes the
transforms trivially unit-testable.
"""

from __future__ import annotations

from dataclasses import dataclass
from itertools import pairwise
import logging
import math
from typing import TYPE_CHECKING, Any

from novapolicy.types import ActionChunk

if TYPE_CHECKING:
    from collections.abc import Iterable

logger = logging.getLogger(__name__)

_MIN_SPACING_STEPS = 2
_MIN_RAMP_STEPS = 2
_MIN_SMOOTHING_STEPS = 2
_SEGMENT_RATIO_EPSILON = 1e-12


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
        action_timestep=chunk.action_timestep,
        seam_backdate_steps=chunk.seam_backdate_steps,
    )


def smooth_action_chunk(
    chunk: ActionChunk,
    *,
    passes: int = 2,
    retained_prefix_steps: int = 0,
) -> ActionChunk:
    """Smooth motion lookaheads without changing active or discrete actions.

    Each pass applies the temporal filter ``[1, 2, 1] / 4`` with replicated
    boundary values. Two passes are equivalent to ``[1, 4, 6, 4, 1] / 16``
    away from chunk boundaries. ``retained_prefix_steps`` restores an already
    active prefix after filtering so trajectory replacements remain exact.

    Joint and TCP sequences are filtered independently. TCP position and
    rotation-vector components are treated component-wise. IO actions, timing,
    and action-timestep metadata are preserved.
    """
    if passes < 0:
        raise ValueError("passes must be non-negative")
    if retained_prefix_steps < 0:
        raise ValueError("retained_prefix_steps must be non-negative")
    if passes == 0 or (not chunk.joints and not chunk.tcp):
        return chunk

    joints = {
        group_id: _smooth_steps(
            steps,
            passes=passes,
            retained_prefix_steps=retained_prefix_steps,
        )
        for group_id, steps in chunk.joints.items()
    }
    tcp = {
        group_id: _smooth_steps(
            steps,
            passes=passes,
            retained_prefix_steps=retained_prefix_steps,
        )
        for group_id, steps in chunk.tcp.items()
    }
    return ActionChunk(
        joints=joints,
        tcp=tcp,
        ios=chunk.ios,
        dt_ms=chunk.dt_ms,
        first_timestamp_ms=chunk.first_timestamp_ms,
        action_timestep=chunk.action_timestep,
        seam_backdate_steps=chunk.seam_backdate_steps,
    )


def _smooth_steps(
    steps: list[list[float]],
    *,
    passes: int,
    retained_prefix_steps: int,
) -> list[list[float]]:
    if len(steps) < _MIN_SMOOTHING_STEPS:
        return steps

    original = [list(step) for step in steps]
    smoothed = original
    for _pass in range(passes):
        previous = smoothed
        smoothed = []
        for index, current in enumerate(previous):
            before = previous[max(0, index - 1)]
            after = previous[min(len(previous) - 1, index + 1)]
            smoothed.append(
                [
                    (left + 2.0 * value + right) / 4.0
                    for left, value, right in zip(before, current, after, strict=True)
                ]
            )

    prefix_length = min(retained_prefix_steps, len(smoothed))
    smoothed[:prefix_length] = original[:prefix_length]
    return smoothed


@dataclass(frozen=True, slots=True)
class InterpolatedActionChunk:
    """Motion with eased endpoint intervals and remapped original indices."""

    motion: ActionChunk
    original_step_indices: dict[str, tuple[int, ...]]
    """Motion group id → new index for every original waypoint index."""


def interpolate_action_chunk_ramps(
    chunk: ActionChunk,
    *,
    accelerate: bool = True,
    brake: bool = True,
    interpolation_steps: int = 3,
) -> InterpolatedActionChunk:
    """Ease into and out of a chunk by subdividing its endpoint intervals.

    The first interval uses quadratic ease-in, while the final interval uses
    quadratic ease-out. If both refer to the same interval, smoothstep is used.
    Every original waypoint remains in the result and ``dt_ms`` is unchanged;
    the added points therefore allocate real time for acceleration and braking.
    """
    if interpolation_steps < _MIN_RAMP_STEPS:
        raise ValueError("interpolation_steps must be at least 2")

    joints: dict[str, list[list[float]]] = {}
    tcp: dict[str, list[list[float]]] = {}
    original_step_indices: dict[str, tuple[int, ...]] = {}

    for group_id, steps in chunk.joints.items():
        interpolated, indices = _interpolate_endpoint_intervals(
            steps,
            accelerate=accelerate,
            brake=brake,
            interpolation_steps=interpolation_steps,
        )
        joints[group_id] = interpolated
        original_step_indices[group_id] = indices

    for group_id, steps in chunk.tcp.items():
        interpolated, indices = _interpolate_endpoint_intervals(
            steps,
            accelerate=accelerate,
            brake=brake,
            interpolation_steps=interpolation_steps,
        )
        tcp[group_id] = interpolated
        original_step_indices[group_id] = indices

    motion = ActionChunk(
        joints=joints,
        tcp=tcp,
        ios=chunk.ios,
        dt_ms=chunk.dt_ms,
        first_timestamp_ms=chunk.first_timestamp_ms,
        action_timestep=chunk.action_timestep,
        seam_backdate_steps=chunk.seam_backdate_steps,
    )
    return InterpolatedActionChunk(motion=motion, original_step_indices=original_step_indices)


def _interpolate_endpoint_intervals(
    steps: list[list[float]],
    *,
    accelerate: bool,
    brake: bool,
    interpolation_steps: int,
) -> tuple[list[list[float]], tuple[int, ...]]:
    if len(steps) < _MIN_RAMP_STEPS:
        return steps, tuple(range(len(steps)))

    result = [list(steps[0])]
    original_indices = [0]
    final_segment = len(steps) - 2
    for segment, (start, end) in enumerate(pairwise(steps)):
        is_acceleration = accelerate and segment == 0
        is_braking = brake and segment == final_segment
        fractions = _ramp_fractions(
            interpolation_steps,
            accelerate=is_acceleration,
            brake=is_braking,
        )
        result.extend(
            [left + (right - left) * fraction for left, right in zip(start, end, strict=True)]
            for fraction in fractions
        )
        original_indices.append(len(result) - 1)
    return result, tuple(original_indices)


def _ramp_fractions(
    interpolation_steps: int,
    *,
    accelerate: bool,
    brake: bool,
) -> list[float]:
    if not accelerate and not brake:
        return [1.0]

    fractions = []
    for index in range(1, interpolation_steps + 1):
        value = index / interpolation_steps
        if accelerate and brake:
            fraction = value * value * (3.0 - 2.0 * value)
        elif accelerate:
            fraction = value * value
        else:
            fraction = 1.0 - (1.0 - value) ** 2
        fractions.append(fraction)
    return fractions


@dataclass(frozen=True, slots=True)
class ConnectedActionChunk:
    """Continuous bridge + policy motion and policy-boundary indices."""

    motion: ActionChunk
    bridge: ActionChunk
    policy_start_steps: dict[str, int]
    """Motion group id → index of policy waypoint zero in ``motion``."""


def connect_action_chunk(
    chunk: ActionChunk,
    states: dict[str, Any],
    *,
    always_anchor: bool = False,
) -> ConnectedActionChunk | None:
    """Prepend same-spacing bridges without stopping at policy waypoint zero.

    The returned motion contains no IO actions. ``policy_start_steps`` marks
    the exact waypoint where callers should fire the original chunk's IO and
    computed actions. Returns ``None`` when no motion group needs a bridge.
    """
    bridge = create_bridge_chunk(chunk, states, always_anchor=always_anchor)
    if bridge is None:
        return None

    connected_joints: dict[str, list[list[float]]] = {}
    connected_tcp: dict[str, list[list[float]]] = {}
    policy_start_steps: dict[str, int] = {}

    for group_id, policy_steps in chunk.joints.items():
        bridge_steps = bridge.joints.get(group_id)
        if bridge_steps:
            policy_start_steps[group_id] = len(bridge_steps) - 1
            connected_joints[group_id] = [*bridge_steps, *policy_steps[1:]]
        else:
            policy_start_steps[group_id] = 0
            connected_joints[group_id] = policy_steps

    for group_id, policy_steps in chunk.tcp.items():
        bridge_steps = bridge.tcp.get(group_id)
        if bridge_steps:
            policy_start_steps[group_id] = len(bridge_steps) - 1
            connected_tcp[group_id] = [*bridge_steps, *policy_steps[1:]]
        else:
            policy_start_steps[group_id] = 0
            connected_tcp[group_id] = policy_steps

    motion = ActionChunk(
        joints=connected_joints,
        tcp=connected_tcp,
        dt_ms=chunk.dt_ms,
    )
    return ConnectedActionChunk(
        motion=motion,
        bridge=bridge,
        policy_start_steps=policy_start_steps,
    )


def create_bridge_chunk(
    chunk: ActionChunk,
    states: dict[str, Any],
    *,
    always_anchor: bool = False,
) -> ActionChunk | None:
    """Create an interpolated chunk from current state to policy step zero.

    A bridge is needed only when reaching the first policy target in one
    ``dt_ms`` interval would exceed the largest spatial interval already
    present in that policy chunk. Its first waypoint holds the observed current
    state, so request transport cannot consume the first movement interval.
    Bridge waypoints use the same ``dt_ms`` and end exactly at policy step zero.
    The returned chunk has no IO actions. Use :func:`connect_action_chunk` to
    prepend it to policy motion without stopping at the boundary.

    With ``always_anchor=True``, a measured-state hold is always prepended. If
    the gap needs no interpolation, the bridge is exactly ``[current, first]``.
    This gives continuously refreshed queues one full ``dt_ms`` interval before
    the first movement target instead of scheduling that target immediately.

    Joint spacing is measured as Euclidean distance in joint space. TCP
    spacing treats translation and rotation-vector distance separately so
    millimetres and radians are never mixed. If a chunk has fewer than two
    policy steps, it provides no spacing reference and no bridge is created.
    """
    bridge_joints: dict[str, list[list[float]]] = {}
    bridge_tcp: dict[str, list[list[float]]] = {}

    for group_id, steps in chunk.joints.items():
        state = states.get(group_id)
        if state is None or not steps or not hasattr(state, "joints"):
            continue
        bridge = _interpolated_bridge(
            list(state.joints),
            steps,
            tcp=False,
            always_anchor=always_anchor,
        )
        if bridge:
            bridge_joints[group_id] = bridge

    for group_id, steps in chunk.tcp.items():
        state = states.get(group_id)
        pose = getattr(state, "pose", None) if state is not None else None
        if pose is None or not steps:
            continue
        current = [*pose.position, *pose.orientation]
        bridge = _interpolated_bridge(
            current,
            steps,
            tcp=True,
            always_anchor=always_anchor,
        )
        if bridge:
            bridge_tcp[group_id] = bridge

    if not bridge_joints and not bridge_tcp:
        return None
    return ActionChunk(joints=bridge_joints, tcp=bridge_tcp, dt_ms=chunk.dt_ms)


def _interpolated_bridge(
    current: list[float],
    policy_steps: list[list[float]],
    *,
    tcp: bool,
    always_anchor: bool,
) -> list[list[float]]:
    if not policy_steps or len(current) != len(policy_steps[0]):
        return []
    if len(policy_steps) < _MIN_SPACING_STEPS:
        return [list(current), list(policy_steps[0])] if always_anchor else []

    if tcp:
        segment_count = _tcp_bridge_segment_count(current, policy_steps)
    else:
        gap = _euclidean_distance(current, policy_steps[0])
        spacing = max(
            _euclidean_distance(previous, following)
            for previous, following in pairwise(policy_steps)
        )
        segment_count = _segment_count(gap, spacing)

    if segment_count <= 1:
        return [list(current), list(policy_steps[0])] if always_anchor else []

    target = policy_steps[0]
    interpolated = [
        [
            start + (end - start) * index / segment_count
            for start, end in zip(current, target, strict=True)
        ]
        for index in range(1, segment_count + 1)
    ]
    return [list(current), *interpolated]


def _tcp_bridge_segment_count(current: list[float], policy_steps: list[list[float]]) -> int:
    translation_gap = _euclidean_distance(current[:3], policy_steps[0][:3])
    rotation_gap = _euclidean_distance(current[3:6], policy_steps[0][3:6])
    translation_spacing = max(
        _euclidean_distance(previous[:3], following[:3])
        for previous, following in pairwise(policy_steps)
    )
    rotation_spacing = max(
        _euclidean_distance(previous[3:6], following[3:6])
        for previous, following in pairwise(policy_steps)
    )
    return max(
        _segment_count(translation_gap, translation_spacing),
        _segment_count(rotation_gap, rotation_spacing),
    )


def _segment_count(gap: float, spacing: float) -> int:
    if gap <= 0.0 or spacing <= 0.0:
        return 1
    return max(1, math.ceil(gap / spacing - _SEGMENT_RATIO_EPSILON))


def _euclidean_distance(left: list[float], right: list[float]) -> float:
    if len(left) != len(right):
        return 0.0
    return math.sqrt(sum((b - a) ** 2 for a, b in zip(left, right, strict=True)))


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
        action_timestep=chunk.action_timestep,
        seam_backdate_steps=chunk.seam_backdate_steps,
    )


@dataclass(frozen=True, slots=True)
class Placement:
    """Where a chunk's first step sits on one session's server timeline.

    ``first_timestamp_ms`` is an exact raw NOVA timestamp, or ``None`` to
    resolve server "now" at yield time. ``timestamp_offset_steps`` shifts that
    timestamp by whole ``dt`` steps: ``+1`` = one step ahead (sequential),
    negative = backdated overlap, ``0`` = exact.
    """

    first_timestamp_ms: int | None
    timestamp_offset_steps: int


def placement(chunk: ActionChunk, *, policy_rate_hz: float) -> Placement:
    """Decide how a chunk is anchored on one session's timeline.

    The "now" component is intentionally NOT resolved here — it is computed at
    yield time in :func:`novapolicy.jogging.waypoints.make_waypoints_request` so the
    anchor cannot go stale while the chunk waits in the session queue. This
    matters most for continuous replacement, whose seam alignment is sensitive to that delay.

    * An explicit ``chunk.first_timestamp_ms`` (>=0) set by the policy wins —
      used verbatim, anchored exactly.
    * Wait-for-chunk (``policy_rate_hz < 0``) — "now", one step ahead.
    * Otherwise (continuous replacement) — "now", backdated by ``seam_backdate_steps`` so
      the step matching the robot's current position lands at "now".
    """
    if chunk.first_timestamp_ms >= 0:
        return Placement(chunk.first_timestamp_ms, 0)
    if policy_rate_hz < 0:
        return Placement(None, 1)
    return Placement(None, -chunk.seam_backdate_steps)
