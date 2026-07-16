"""Unit tests for timestamp-aligned action queue behavior."""

from __future__ import annotations

import numpy as np
import pytest

from novapolicy.action_queue import AsyncQueueAggregation, TimestampedActionQueue


def test_weighted_aggregation_blends_old_and_new_predictions() -> None:
    queue = TimestampedActionQueue(aggregation=AsyncQueueAggregation.WEIGHTED_AVERAGE)
    queue.merge([(4, np.asarray([2.0], dtype=np.float32))])

    assert queue.merge([(4, np.asarray([10.0], dtype=np.float32))])
    assert queue.entries[0][1][0] == pytest.approx(7.6)


def test_current_and_two_successors_are_frozen() -> None:
    queue = TimestampedActionQueue(
        aggregation=AsyncQueueAggregation.WEIGHTED_AVERAGE,
        frozen_steps=3,
    )
    queue.merge(
        (timestep, np.asarray([float(timestep)], dtype=np.float32)) for timestep in range(5)
    )
    queue.consume()
    queue.publish(
        (timestep, np.asarray([float(timestep + 10)], dtype=np.float32)) for timestep in range(1, 4)
    )

    assert queue.merge((timestep, np.asarray([10.0], dtype=np.float32)) for timestep in range(1, 5))
    assert [action[0] for _timestep, action in queue.entries] == pytest.approx([
        11.0,
        12.0,
        13.0,
        8.2,
    ])


def test_average_aggregation_is_a_true_running_mean_per_timestep() -> None:
    queue = TimestampedActionQueue(aggregation=AsyncQueueAggregation.AVERAGE)
    queue.merge([(4, np.asarray([2.0], dtype=np.float32))])

    assert queue.merge([(4, np.asarray([10.0], dtype=np.float32))])
    assert queue.merge([(4, np.asarray([12.0], dtype=np.float32))])

    assert queue.entries[0][1][0] == pytest.approx((2.0 + 10.0 + 12.0) / 3.0)
    assert queue.prediction_counts == {4: 3}


def test_synchronization_drops_actions_elapsed_on_nova() -> None:
    queue = TimestampedActionQueue()
    queue.merge(
        (timestep, np.asarray([float(timestep)], dtype=np.float32)) for timestep in range(2, 5)
    )

    assert queue.synchronize(4) == 4
    assert queue.latest_timestep == 3
    assert [timestep for timestep, _action in queue.entries] == [4]
