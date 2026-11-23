"""Hold a live observation against the range a policy was trained on.

Declared units are an assertion by whoever wrote the schema; a checkpoint's statistics can
settle it. Two thresholds, deliberately different in kind: sitting outside the
demonstrated *range* only warns, because a robot parked somewhere the demonstrations never
visited is normal; being an order of *magnitude* larger than anything in training raises,
because that is not a pose, it is a unit (radians read as degrees is ~57x).
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from collections.abc import Iterable, Mapping, Sequence

logger = logging.getLogger(__name__)

WARN_MARGIN = 1.0
"""A value this many training spans outside the range is worth mentioning."""

FAIL_SCALE = 5.0
"""A value this many times larger than anything in training is a unit error. Scaling the
failure threshold by span instead was measured wrong against a live UR10e: a joint that
barely moved during training (0.022 rad) got a +/-0.09 rad window, and the arm's own park
pose was reported as a unit mismatch."""


def classify(values: Iterable[tuple[str, float, float, float]]) -> tuple[list[str], list[str]]:
    """Split ``(name, value, low, high)`` into implausible values and merely unusual ones."""
    problems: list[str] = []
    warnings: list[str] = []
    for name, value, low, high in values:
        span = high - low
        magnitude = max(abs(low), abs(high))
        if span <= 0 and magnitude <= 0:
            continue
        note = f"{name!r} is {float(value):.4g}, training range [{low:.4g}, {high:.4g}]"
        if magnitude > 0 and abs(value) > FAIL_SCALE * magnitude:
            problems.append(note)
        elif span > 0 and not (low - WARN_MARGIN * span <= value <= high + WARN_MARGIN * span):
            warnings.append(note)
    return problems, warnings


def check(values: Iterable[tuple[str, float, float, float]]) -> None:
    """Warn on unusual values, raise ``ValueError`` on implausible ones."""
    problems, warnings = classify(values)
    for note in warnings:
        logger.warning(
            "Observation outside the training distribution: %s. Normal at a start pose the "
            "demonstrations never visited; suspicious otherwise.",
            note,
        )
    if problems:
        detail = "\n  - ".join(problems)
        msg = (
            "Observations are implausibly far outside what this checkpoint was trained on:"
            f"\n  - {detail}\n"
            "This is what a unit mismatch looks like — check whether the dataset was "
            "recorded in different units than NOVA reports, and declare the conversion "
            "with ops= on the observation."
        )
        raise ValueError(msg)


def check_vectors(
    values: Mapping[str, Sequence[object]],
    ranges: Mapping[str, Sequence[tuple[float, float] | None]],
    labels: Mapping[str, Sequence[str]] | None = None,
) -> None:
    """:func:`check` over observation vectors keyed like the checkpoint's ranges, each
    dimension named ``labels[key][i]`` or ``key[i]``. A vector as wide as its range only (a
    width that disagrees is the schema check's failure, not this one's); a dimension without
    bounds or without a numeric value is skipped."""
    bounded: list[tuple[str, float, float, float]] = []
    for key, vector in values.items():
        bounds = ranges.get(key)
        if bounds is None or len(bounds) != len(vector):
            continue
        names = (labels or {}).get(key) or [f"{key}[{i}]" for i in range(len(vector))]
        bounded += [
            (name, float(value), *bound)
            for name, value, bound in zip(names, vector, bounds, strict=True)
            if bound is not None and isinstance(value, (int, float)) and not isinstance(value, bool)
        ]
    check(bounded)
