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
    ) -> None:
        self._aggregation = AsyncQueueAggregation(aggregation)
        self._actions: list[tuple[int, NDArray[np.float32]]] = []
        self._prediction_counts: dict[int, int] = {}
        self._published: dict[int, NDArray[np.float32]] = {}
        self._latest_timestep = -1

    @property
    def actions(self) -> list[tuple[int, NDArray[np.float32]]]:
        """The queued ``(timestep, action)`` pairs in timestep order (a copy)."""
        return list(self._actions)

    def clear(self) -> None:
        self._actions.clear()
        self._prediction_counts.clear()
        self._published.clear()
        self._latest_timestep = -1

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

    def merge(
        self,
        predictions: Iterable[tuple[int, NDArray[np.float32]]],
        *,
        frozen_steps: int = 0,
    ) -> bool:
        """Merge a timestamped prediction into the replaceable future queue; the first
        ``frozen_steps`` after the last executed step keep their published values."""
        decoded = list(predictions)
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
            if timestep <= self._latest_timestep + frozen_steps:
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

    def predecessor(self, timestep: int) -> NDArray[np.float32] | None:
        return self._published.get(timestep - 1)

    def publish(self, actions: Iterable[tuple[int, NDArray[np.float32]]]) -> None:
        """Remember the exact values submitted to the controller."""
        self._published = dict(actions)
