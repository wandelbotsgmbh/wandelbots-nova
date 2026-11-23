"""Hold a live observation against the range a policy was trained on (ADR 0014).

Outside the training range warns; :data:`FAIL_SCALE` times larger than anything in training raises.
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
"""A value this many times larger in magnitude than anything in training is a unit error."""


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
    """:func:`check` over observation vectors keyed like the checkpoint's ranges.

    Dimensions are named ``labels[key][i]`` or ``key[i]``; vectors of another width than their
    range, and dimensions without bounds or a numeric value, are skipped.
    """
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
