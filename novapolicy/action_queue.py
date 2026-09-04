"""Reusable timestamp-aligned action queue for asynchronous policy clients."""

from __future__ import annotations

from enum import StrEnum
from typing import TYPE_CHECKING, cast

if TYPE_CHECKING:
    from collections.abc import Iterable

    import numpy as np
    from numpy.typing import NDArray


class AsyncQueueAggregation(StrEnum):
    """How predictions targeting the same future timestep are merged."""

    WEIGHTED_AVERAGE = "weighted_average"
    AVERAGE = "average"

    @property
    def old_action_weight(self) -> float:
        """Weight assigned to the queued action; the new action gets the remainder."""
        return 0.3


class TimestampedActionQueue:
    """Aggregate and consume flat actions keyed by absolute policy timestep."""

    def __init__(
        self,
        *,
        aggregation: AsyncQueueAggregation = AsyncQueueAggregation.WEIGHTED_AVERAGE,
        frozen_steps: int = 0,
    ) -> None:
        self._aggregation = AsyncQueueAggregation(aggregation)
        self._frozen_steps = frozen_steps
        self._actions: list[tuple[int, NDArray[np.float32]]] = []
        self._prediction_counts: dict[int, int] = {}
        self._published: dict[int, NDArray[np.float32]] = {}
        self._latest_timestep = -1
        self._chunk_size = -1

    @property
    def empty(self) -> bool:
        return not self._actions

    @property
    def latest_timestep(self) -> int:
        return self._latest_timestep

    def clear(self) -> None:
        self._actions.clear()
        self._prediction_counts.clear()
        self._published.clear()
        self._latest_timestep = -1
        self._chunk_size = -1

    def synchronize(self, timestep: int) -> int:
        """Drop queued actions before ``timestep`` and return the skipped count."""
        if timestep <= self._latest_timestep + 1:
            return 0
        previous = self._latest_timestep
        self._latest_timestep = timestep - 1
        self._actions = [
            (queued_timestep, action)
            for queued_timestep, action in self._actions
            if queued_timestep >= timestep
        ]
        retained = {queued_timestep for queued_timestep, _action in self._actions}
        self._prediction_counts = {
            queued_timestep: count
            for queued_timestep, count in self._prediction_counts.items()
            if queued_timestep in retained
        }
        return timestep - previous - 1

    def consume(self) -> tuple[int, NDArray[np.float32]]:
        """Remove and return the next action in timestep order."""
        timestep, action = self._actions.pop(0)
        self._prediction_counts.pop(timestep, None)
        self._latest_timestep = timestep
        return timestep, action

    def refill_needed(self, threshold: float) -> bool:
        if self._chunk_size <= 0:
            return True
        return len(self._actions) / self._chunk_size < threshold

    def merge(self, predictions: Iterable[tuple[int, NDArray[np.float32]]]) -> bool:
        """Merge a timestamped prediction into the replaceable future queue."""
        decoded = list(predictions)
        self._chunk_size = max(self._chunk_size, len(decoded))
        current = dict(self._actions)
        future: dict[int, NDArray[np.float32]] = {}
        future_counts: dict[int, int] = {}

        for timestep, new_action in decoded:
            if timestep <= self._latest_timestep:
                continue
            if timestep not in current:
                future[timestep] = new_action
                future_counts[timestep] = 1
                continue

            previous_count = self._prediction_counts.get(timestep, 1)
            if timestep <= self._latest_timestep + self._frozen_steps:
                future[timestep] = self._published.get(timestep, current[timestep])
                future_counts[timestep] = previous_count
            elif self._aggregation is AsyncQueueAggregation.AVERAGE:
                future[timestep] = cast(
                    "NDArray[np.float32]",
                    (previous_count * current[timestep] + new_action) / (previous_count + 1),
                )
                future_counts[timestep] = previous_count + 1
            else:
                old_weight = self._aggregation.old_action_weight
                future[timestep] = cast(
                    "NDArray[np.float32]",
                    old_weight * current[timestep] + (1.0 - old_weight) * new_action,
                )
                future_counts[timestep] = previous_count + 1

        self._actions = sorted(future.items())
        self._prediction_counts = future_counts
        return bool(future)

    def preview(
        self, selected: tuple[int, NDArray[np.float32]]
    ) -> list[tuple[int, NDArray[np.float32]]]:
        """Return the selected action followed by the queued lookahead."""
        return [selected, *self._actions]

    def predecessor(self, timestep: int) -> NDArray[np.float32] | None:
        return self._published.get(timestep - 1)

    def publish(self, actions: Iterable[tuple[int, NDArray[np.float32]]]) -> None:
        """Remember the exact values submitted to the controller."""
        self._published = dict(actions)
