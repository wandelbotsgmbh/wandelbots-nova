"""``Smoothing``: smooth a plan's motion before it is anchored (contract T13)."""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import TYPE_CHECKING, ClassVar

if TYPE_CHECKING:
    from novapolicy.pipeline.base import ModuleContext
    from novapolicy.transport.waypoints import PendingChunk

_MIN_SMOOTHING_STEPS = 2


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


@dataclass
class Smoothing:
    """Chunk module: smooth a plan's motion with ``passes`` of the filter ``[1, 2, 1] / 4``.

    Steps already sent (``retained_prefix_steps``) keep the values sent for their timesteps.
    Joint and TCP values are filtered per component; IO, timing and computed-action values are
    untouched; chunks under three steps pass unchanged. Put it before the ``Bridge``.
    """

    stage: ClassVar[str] = "smooth"  # its place among the chunk modules (CHUNK_STAGES)

    passes: int = 2
    _sent: dict[int, list[float]] = field(default_factory=dict, init=False, repr=False)
    """The values last sent, by policy timestep."""

    def __post_init__(self) -> None:
        if self.passes < 0:
            raise ValueError("Smoothing needs a non-negative number of passes")

    def on_chunk(
        self,
        chunk: PendingChunk,
        context: ModuleContext,  # ruff: ignore[unused-method-argument] (the protocol's)
    ) -> PendingChunk:
        if self.passes == 0:
            return chunk
        retained = min(chunk.retained_prefix_steps, len(chunk.steps))
        first = chunk.action_timestep
        steps = [list(step) for step in chunk.steps]
        if first >= 0:
            for i in range(retained):
                sent = self._sent.get(first + i)
                if sent is not None and len(sent) == len(steps[i]):
                    steps[i] = list(sent)
        smoothed = smooth_steps(steps, passes=self.passes, retained_prefix_steps=retained)
        if first >= 0:
            self._sent = {first + i: step for i, step in enumerate(smoothed)}
        return replace(chunk, steps=smoothed)
