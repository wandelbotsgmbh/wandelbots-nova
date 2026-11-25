"""``Bridge``: bridge from the measured position to the policy's first step (contract T13)."""

from __future__ import annotations

from dataclasses import dataclass, replace
from itertools import pairwise
import math
from typing import TYPE_CHECKING, ClassVar

from novapolicy.mapping.poses import unwrap_tcp_rotations

if TYPE_CHECKING:
    from novapolicy.pipeline.base import ModuleContext
    from novapolicy.transport.action_chunks import PendingChunk

_MIN_SPACING_STEPS = 2
_SEGMENT_RATIO_EPSILON = 1e-12


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
class Bridge:
    """Chunk module: bridge from the measured position to the policy's first step.

    A hold on the measured position, then steps at the chunk's spacing, subdivided as far as its
    largest step needs; ``always_anchor`` adds the hold even without subdivision. Chunks with
    ``first_timestamp_ms`` or already at the robot pass unchanged. In continuous execution a
    bridged chunk is dated at the send (T2). IO stays with the policy's first step; TCP
    rotations are unwrapped against the measured pose.
    """

    stage: ClassVar[str] = "anchor"  # its place among the chunk modules (CHUNK_STAGES)

    always_anchor: bool = False

    def on_chunk(self, chunk: PendingChunk, context: ModuleContext) -> PendingChunk:
        if chunk.first_timestamp_ms is not None or context.position is None:
            return chunk
        tcp = context.mode == "cartesian"
        steps = unwrap_tcp_rotations(chunk.steps, context.position[3:6]) if tcp else chunk.steps
        bridge = bridge_steps(context.position, steps, tcp=tcp, always_anchor=self.always_anchor)
        if not bridge:
            return chunk
        shift = len(bridge) - 1
        when_due = chunk.when_due
        if when_due is not None:
            when_due = (when_due[0] + shift, when_due[1])
        bridged = replace(
            chunk,
            steps=[*bridge, *steps[1:]],
            caller_step_count=len(bridge) + len(steps) - 1,
            when_due=when_due,
            start_step=chunk.start_step + shift,
        )
        if chunk.lead and chunk.observed_at_ms is not None:
            # Dated by its observation (continuous execution): the bridge starts now instead.
            bridged = replace(bridged, lead=False, observed_at_ms=None, timestamp_offset_steps=0)
        return bridged
