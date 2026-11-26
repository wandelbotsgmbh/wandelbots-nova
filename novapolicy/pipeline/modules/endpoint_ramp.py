"""``EndpointRamp``: ease into and out of each chunk (sequential execution)."""

from __future__ import annotations

from dataclasses import dataclass, replace
from itertools import pairwise
from typing import TYPE_CHECKING, ClassVar

from novapolicy.mapping.poses import unwrap_tcp_rotations

if TYPE_CHECKING:
    from novapolicy.pipeline.base import ModuleContext
    from novapolicy.transport.action_chunks import PendingChunk

_MIN_RAMP_STEPS = 2


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


@dataclass(frozen=True, slots=True)
class EndpointRamp:
    """Chunk module: ease into and out of each chunk by subdividing its first and last interval.

    Each endpoint interval becomes ``interpolation_steps`` intervals of the chunk's spacing,
    quadratic ease-in / ease-out (a smoothstep when they coincide). Every original step and its
    IO is kept. For sequential execution, where chunks start and end at rest.
    """

    stage: ClassVar[str] = "shape"  # its place among the chunk modules (CHUNK_STAGES)

    interpolation_steps: int = 3
    """Intervals replacing each endpoint interval. Must be at least two."""

    def __post_init__(self) -> None:
        if self.interpolation_steps < 2:  # ruff: ignore[magic-value-comparison] (one interval is no ramp)
            raise ValueError("interpolation_steps must be at least 2")

    def on_chunk(self, chunk: PendingChunk, context: ModuleContext) -> PendingChunk:
        steps, indices = ramp_steps(
            chunk.steps,
            interpolation_steps=self.interpolation_steps,
            tcp=context.mode == "cartesian",
        )
        if len(steps) == len(chunk.steps):
            return chunk
        when_due = chunk.when_due
        if when_due is not None:
            when_due = (indices[min(when_due[0], len(indices) - 1)], when_due[1])
        return replace(
            chunk,
            steps=steps,
            caller_step_count=len(steps),
            when_due=when_due,
            start_step=indices[min(chunk.start_step, len(indices) - 1)],
        )
