"""Resolve time/distance path triggers into concrete path-parameter locations.

Time- and distance-based :class:`~nova.actions.path_trigger.PathTrigger`s cannot
be turned into a path location until the trajectory is planned: they are defined
relative to a motion action but their concrete location depends on the planned
time profile (for time) and the planned Cartesian path length (for distance).

This module is pure: it takes the already-planned per-sample ``times`` /
``locations`` (and, for distance, the per-sample TCP positions) and returns a
mapping from write-action index to resolved float location. The mapping is keyed
by the position of the write action within the *write-action subsequence* of the
trajectory, matching the order produced by
:meth:`nova.actions.container.CombinedActions.to_set_io`.
"""

from __future__ import annotations

import logging
from typing import Sequence

import numpy as np

from nova.actions.container import CombinedActions
from nova.actions.io import WriteAction
from nova.actions.path_trigger import (
    DistanceTrigger,
    PathParameterTrigger,
    PathTrigger,
    TimeTrigger,
    TriggerReference,
)

logger = logging.getLogger(__name__)


def _interp(x: float, xp: np.ndarray, fp: np.ndarray) -> float:
    """Linear interpolation of ``fp`` at ``x`` over a non-decreasing ``xp``.

    ``np.interp`` clamps to the endpoints outside ``[xp[0], xp[-1]]`` and copes
    with flat (repeated) ``xp`` regions, which both occur in planned
    trajectories (e.g. wait segments hold a constant location).
    """
    return float(np.interp(x, xp, fp))


def _cumulative_arclength(positions: Sequence[Sequence[float]]) -> np.ndarray:
    """Cumulative Cartesian distance [mm] along the per-sample TCP positions."""
    pts = np.asarray(positions, dtype=float)
    if pts.ndim != 2 or len(pts) == 0:
        return np.zeros(len(pts))
    step = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(step)])


def _clip_to_segment(resolved: float, lower: float, upper: float, trigger: PathTrigger) -> float:
    """Clamp ``resolved`` to the anchor segment ``[lower, upper]``, warning on overshoot."""
    clipped = float(np.clip(resolved, lower, upper))
    if not np.isclose(clipped, resolved):
        logger.warning(
            "Path trigger %r resolved to location %.4f outside its motion segment "
            "[%.1f, %.1f]; clamping to the segment boundary at %.4f.",
            trigger,
            resolved,
            lower,
            upper,
            clipped,
        )
    return clipped


def _resolve_relative(
    domain: np.ndarray,
    locations: np.ndarray,
    lower: float,
    upper: float,
    offset: float,
    reference: TriggerReference,
    trigger: PathTrigger,
) -> float:
    """Resolve a time/distance trigger by mapping an offset in ``domain`` to a location.

    ``domain`` is the planned quantity the offset is measured in (per-sample times
    for time triggers, cumulative arc length for distance triggers), parallel to
    ``locations``. Offsets that leave the anchor segment are clamped to its
    boundary and a warning is logged.
    """
    domain_lower = _interp(lower, locations, domain)
    domain_upper = _interp(upper, locations, domain)
    if reference is TriggerReference.PREVIOUS:
        target = domain_lower + offset
    else:
        target = domain_upper - offset
    clamped = float(np.clip(target, domain_lower, domain_upper))
    if not np.isclose(clamped, target):
        logger.warning(
            "Path trigger %r resolved outside its motion segment [%.1f, %.1f]; "
            "clamping to the segment boundary.",
            trigger,
            lower,
            upper,
        )
    return _interp(clamped, domain, locations)


def _resolve_one(
    trigger: PathTrigger,
    anchor: int,
    times: np.ndarray,
    locations: np.ndarray,
    cumulative_s: np.ndarray | None,
) -> float | None:
    """Resolve a single trigger to a float location, or ``None`` to keep the anchor."""
    lower = float(anchor)
    upper = float(anchor + 1)

    if isinstance(trigger, PathParameterTrigger):
        return _clip_to_segment(lower + trigger.value, lower, upper, trigger)

    if isinstance(trigger, TimeTrigger):
        return _resolve_relative(
            times, locations, lower, upper, trigger.seconds, trigger.reference, trigger
        )

    if isinstance(trigger, DistanceTrigger):
        if cumulative_s is None:
            logger.warning(
                "Distance trigger could not be resolved (no TCP positions available); "
                "falling back to the motion boundary at location %s.",
                anchor,
            )
            return None
        return _resolve_relative(
            cumulative_s, locations, lower, upper, trigger.millimeters, trigger.reference, trigger
        )

    return None


def resolve_trigger_locations(
    combined_actions: CombinedActions,
    times: Sequence[float],
    locations: Sequence[float],
    tcp_positions: Sequence[Sequence[float]] | None = None,
) -> dict[int, float]:
    """Resolve all triggered write actions to float locations.

    Args:
        combined_actions: The actions whose write actions may carry triggers.
        times: Per-sample times [s] of the planned trajectory.
        locations: Per-sample motion-index locations of the planned trajectory
            (parallel to ``times``).
        tcp_positions: Per-sample TCP positions [mm] (parallel to ``times``),
            required only when distance triggers are present.

    Returns:
        Mapping from write-action subsequence index to resolved float location.
        Only entries for actions carrying a trigger are included; an empty
        result means there is nothing to override.
    """
    write_actions = [a for a in combined_actions.actions if isinstance(a.action, WriteAction)]
    triggered = [
        (index, action_location.action, action_location.path_parameter)
        for index, action_location in enumerate(write_actions)
        if isinstance(action_location.action, WriteAction)
        and action_location.action.trigger is not None
    ]
    if not triggered:
        return {}

    times_arr = np.asarray(times, dtype=float)
    locations_arr = np.asarray(locations, dtype=float)
    cumulative_s = _cumulative_arclength(tcp_positions) if tcp_positions is not None else None

    overrides: dict[int, float] = {}
    for index, write, path_parameter in triggered:
        trigger = write.trigger
        if trigger is None:
            continue
        anchor = int(round(path_parameter))
        resolved = _resolve_one(trigger, anchor, times_arr, locations_arr, cumulative_s)
        if resolved is not None:
            overrides[index] = resolved
    return overrides
