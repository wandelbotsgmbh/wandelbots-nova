"""Pure action-chunk transforms used by the executor and the pipeline modules."""

from __future__ import annotations

from dataclasses import dataclass
from itertools import pairwise
import logging
import math
from typing import TYPE_CHECKING, Any, cast

import numpy as np
from scipy.spatial.transform import Rotation

from novapolicy.jogging.clock import ServerMs
from novapolicy.types import ActionChunk, ContinuousExecution

if TYPE_CHECKING:
    from collections.abc import Iterable

    from novapolicy.types import ExecutionMode

logger = logging.getLogger(__name__)

_MIN_SPACING_STEPS = 2
_MIN_RAMP_STEPS = 2
_MIN_SMOOTHING_STEPS = 2
_SEGMENT_RATIO_EPSILON = 1e-12
_ROTVEC_EPSILON = 1e-12


def _max_steps(chunk: ActionChunk) -> int:
    """Longest step sequence across all joint and TCP motion groups."""
    return max(
        (len(steps) for steps in (*chunk.joints.values(), *chunk.tcp.values())),
        default=0,
    )


def trim_chunk(chunk: ActionChunk, n: int) -> ActionChunk:
    """Trim an action chunk to its first ``n`` steps; ``n <= 0`` keeps all (contract M6).

    Warns when the policy returned fewer steps than ``n``.
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

    return chunk.model_copy(
        update={
            "joints": trimmed_joints,
            "tcp": trimmed_tcp,
            "values": {key: rows[:n] for key, rows in chunk.values.items()},
            "retained_prefix_steps": min(chunk.retained_prefix_steps, n),
        }
    )


def smooth_steps(
    steps: list[list[float]], *, passes: int = 2, retained_prefix_steps: int = 0
) -> list[list[float]]:
    """One motion group's steps through ``passes`` of ``[1, 2, 1] / 4`` with replicated ends.

    The first ``retained_prefix_steps`` stay exact; fewer than three steps pass unchanged.
    """
    if passes < 0 or retained_prefix_steps < 0:
        raise ValueError("passes and retained_prefix_steps must be non-negative")
    if passes == 0:
        return steps
    return _smooth_steps(steps, passes=passes, retained_prefix_steps=retained_prefix_steps)


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
            smoothed.append([
                (left + 2.0 * value + right) / 4.0
                for left, value, right in zip(before, current, after, strict=True)
            ])

    prefix_length = min(retained_prefix_steps, len(smoothed))
    smoothed[:prefix_length] = original[:prefix_length]
    return smoothed


def ramp_steps(
    steps: list[list[float]], *, interpolation_steps: int = 3, tcp: bool = False
) -> tuple[list[list[float]], tuple[int, ...]]:
    """One motion group's steps eased in and out (see ``EndpointRamp``), and the new index of
    every original step; TCP rotation vectors are unwrapped first."""
    if interpolation_steps < _MIN_RAMP_STEPS:
        raise ValueError("interpolation_steps must be at least 2")
    if tcp and steps:
        steps = unwrap_tcp_rotations(steps, steps[0][3:6])
    return _interpolate_endpoint_intervals(
        steps, accelerate=True, brake=True, interpolation_steps=interpolation_steps
    )


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


_TCP_WIDTH = 6
"""Values of a NOVA TCP pose: [x, y, z, rx, ry, rz]."""


def unwrap_tcp_rotations(steps: list[list[float]], reference: list[float]) -> list[list[float]]:
    """Rewrite each TCP step's rotation vector as the equivalent one nearest its predecessor.

    ``angle * axis`` equals ``(angle + 2 pi k) * axis``; near pi measurements flip between ``v``
    and about ``-v``, and interpolating across that would turn the wrist half a turn.
    ``reference`` anchors the first step.
    """
    out: list[list[float]] = []
    previous = list(reference)
    for step in steps:
        rotvec = _equivalent_rotvec_near(step[3:6], previous)
        out.append([*step[:3], *rotvec, *step[6:]])
        previous = rotvec
    return out


def _equivalent_rotvec_near(rotvec: list[float], reference: list[float]) -> list[float]:
    angle = math.sqrt(sum(value * value for value in rotvec))
    if angle < _ROTVEC_EPSILON:
        # The identity has no axis of its own: its equivalents are the full turns about any
        # axis, so take the one along the reference nearest to it.
        reference_angle = math.sqrt(sum(value * value for value in reference))
        if reference_angle < _ROTVEC_EPSILON:
            return list(rotvec)
        turns = round(reference_angle / (2.0 * math.pi))
        scale = 2.0 * math.pi * turns / reference_angle
        return [v + r * scale for v, r in zip(rotvec, reference, strict=True)]
    axis = [value / angle for value in rotvec]
    # Equivalent vectors lie on the axis line at angle + 2 pi k; pick the one
    # nearest the reference's projection onto that line.
    projection = sum(a * r for a, r in zip(axis, reference, strict=True))
    turns = round((projection - angle) / (2.0 * math.pi))
    return [a * (angle + 2.0 * math.pi * turns) for a in axis]


def bridge_steps(
    current: list[float],
    steps: list[list[float]],
    *,
    tcp: bool = False,
    always_anchor: bool = False,
) -> list[list[float]]:
    """One motion group's bridge from ``current`` to ``steps[0]`` (see ``Bridge``).

    Empty when one step of the policy's largest spacing reaches it, unless ``always_anchor``.
    Joint spacing is Euclidean; TCP spacing compares translation and rotation separately.
    """
    if tcp and steps:
        steps = unwrap_tcp_rotations(steps, current[3:6])
    return _interpolated_bridge(list(current), steps, tcp=tcp, always_anchor=always_anchor)


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


@dataclass(frozen=True, slots=True)
class TcpDelta:
    """How a relative TCP action's steps apply (see ``Action.tcp``)."""

    accumulate: bool = False
    """True: each step applies to the pose the previous step reached, not the observed one."""
    use_tool_coordinate_system: bool = False
    """True: offsets in the tool coordinate system at the step's start pose; False: in world."""


def apply_relative_mode(
    chunk: ActionChunk,
    states: dict[str, Any],
    relative_mgs: Iterable[str],
    tcp_deltas: dict[str, TcpDelta] | None = None,
) -> ActionChunk:
    """Convert relative action targets of ``relative_mgs`` to absolute ones (contract M5).

    TCP steps compose as ``tcp_deltas`` declares and are unwrapped onto the starting rotation.
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

        # Joint deltas accumulate: step i is current + sum(deltas[0..i]).
        if mg_id in new_joints:
            running = list(state.joints)
            abs_steps = []
            for step in new_joints[mg_id]:
                running = [r + d for r, d in zip(running, step, strict=True)]
                abs_steps.append(list(running))
            new_joints[mg_id] = abs_steps

        if mg_id in new_tcp and hasattr(state, "pose") and state.pose is not None:
            start = [*state.pose.position, *state.pose.orientation]
            delta = (tcp_deltas or {}).get(mg_id, TcpDelta())
            new_tcp[mg_id] = compose_tcp_deltas(start, new_tcp[mg_id], delta)

    return ActionChunk(
        joints=new_joints,
        tcp=new_tcp,
        ios=chunk.ios,
        values=chunk.values,
        dt_ms=chunk.dt_ms,
        first_timestamp_ms=chunk.first_timestamp_ms,
        action_timestep=chunk.action_timestep,
    )


def compose_tcp_deltas(
    start: list[float], steps: list[list[float]], delta: TcpDelta
) -> list[list[float]]:
    """Absolute TCP targets ``[x, y, z, rx, ry, rz]`` (mm, rotation vector) from offsets.

    A rotation offset turns the tool about the TCP: pre-multiplied in world axes,
    post-multiplied in the tool coordinate system.
    """
    widths = sorted({len(step) for step in steps if len(step) != _TCP_WIDTH})
    if widths:
        msg = (
            f"Relative TCP offsets must be [x, y, z, rx, ry, rz] (6 values), got {widths}: "
            "declare the policy's rotation representation with orientation_type"
        )
        raise ValueError(msg)
    p0, r0 = np.asarray(start[:3], dtype=float), Rotation.from_rotvec(start[3:6])
    p, r = p0, r0
    out: list[list[float]] = []
    for step in steps:
        base_p, base_r = (p, r) if delta.accumulate else (p0, r0)
        dp, rv = np.asarray(step[:3], dtype=float), np.asarray(step[3:6], dtype=float)
        if delta.use_tool_coordinate_system:
            p = base_p + base_r.apply(dp)
            r = cast("Rotation", base_r * Rotation.from_rotvec(rv))
        else:
            p = base_p + dp
            r = cast("Rotation", Rotation.from_rotvec(rv) * base_r)
        out.append([*p.tolist(), *r.as_rotvec().tolist()])
    return unwrap_tcp_rotations(out, start[3:6])


def is_single_step(chunk: ActionChunk) -> bool:
    """Whether every motion group of ``chunk`` gets exactly one waypoint."""
    groups = [steps for steps in (*chunk.joints.values(), *chunk.tcp.values()) if steps]
    return bool(groups) and all(len(steps) == 1 for steps in groups)


@dataclass(frozen=True, slots=True)
class Placement:
    """Where a chunk's first step sits on the session clock: a timestamp, or ``None`` for now."""

    first_timestamp_ms: ServerMs | None
    lead: bool = False
    """Placed by the lead rule (contract T1)."""


def placement(chunk: ActionChunk, *, execution: ExecutionMode) -> Placement:
    """Decide how a chunk is anchored on one session's timeline (contracts T1, T2).

    The lead rule, except: an explicit ``first_timestamp_ms`` is used verbatim, and in
    continuous execution a chunk with its own timestep, or several steps without a spacing,
    starts at "now" at the send.
    """
    if chunk.first_timestamp_ms >= 0:
        return Placement(ServerMs(chunk.first_timestamp_ms))
    if not isinstance(execution, ContinuousExecution):
        return Placement(None, lead=True)
    if chunk.action_timestep < 0 and (chunk.dt_ms > 0 or is_single_step(chunk)):
        return Placement(None, lead=True)
    return Placement(None)
