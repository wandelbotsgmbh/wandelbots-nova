"""Resolve path-triggered write actions into the ``set_outputs`` IO overlay.

Write actions without a trigger fire at the integer motion boundary given by their
position in the action list (``ActionLocation.path_parameter``). Write actions with an
``at`` trigger (see :mod:`nova.actions.path_trigger`) fire at a point *within* the
segment that follows that boundary; where exactly depends on the planned trajectory:

- ``path_fraction`` needs nothing but the anchor: ``anchor + value``.
- ``time`` is mapped through the planned per-sample ``times`` / ``locations``.
- ``distance`` is mapped through the cumulative Cartesian arc length of the per-sample
  TCP positions, which the caller obtains via forward kinematics.

This module is pure: it takes the planned per-sample data and returns the complete
``SetIO`` list in write-action order, matching
:meth:`nova.actions.container.CombinedActions.to_set_io`.
"""

from __future__ import annotations

import logging
from typing import Sequence

import numpy as np

from nova import api
from nova.actions.container import CombinedActions
from nova.actions.io import WriteAction
from nova.actions.path_trigger import (
    AtReference,
    AtTrigger,
    DistanceTrigger,
    PathFractionTrigger,
    TimeTrigger,
)

logger = logging.getLogger(__name__)


def has_path_triggers(combined_actions: CombinedActions) -> bool:
    """Whether any write action in ``combined_actions`` carries an ``at`` trigger."""
    return any(trigger is not None for _, trigger in _write_triggers(combined_actions))


def has_distance_triggers(combined_actions: CombinedActions) -> bool:
    """Whether any write action carries a distance trigger (needs TCP positions)."""
    return any(
        isinstance(trigger, DistanceTrigger) for _, trigger in _write_triggers(combined_actions)
    )


def resolve_set_outputs(
    combined_actions: CombinedActions,
    times: Sequence[float],
    locations: Sequence[float],
    tcp_positions: Sequence[Sequence[float]] | None = None,
) -> list[api.models.SetIO]:
    """Build the ``set_outputs`` overlay, resolving path triggers against the plan.

    Args:
        combined_actions: The actions whose write actions may carry ``at`` triggers.
        times: Per-sample times [s] of the planned trajectory.
        locations: Per-sample motion-index locations of the planned trajectory
            (parallel to ``times``).
        tcp_positions: Per-sample TCP positions [mm] (parallel to ``times``). Only
            needed when distance triggers are present; without it those fall back
            to their anchor boundary with a warning.

    Returns:
        One ``SetIO`` per write action, in action-list order. Writes without a
        trigger keep their motion-boundary location, so with no triggers present the
        result equals :meth:`CombinedActions.to_set_io`.
    """
    times_arr = np.asarray(times, dtype=float)
    locations_arr = np.asarray(locations, dtype=float)
    arclength = _cumulative_arclength(tcp_positions) if tcp_positions is not None else None
    n_motions = len(combined_actions.motions)

    result: list[api.models.SetIO] = []
    for action_location in combined_actions.actions:
        write = action_location.action
        if not isinstance(write, WriteAction):
            continue
        location = action_location.path_parameter
        if write.at is not None:
            anchor = int(round(location))
            location = _resolve(write.at, anchor, n_motions, times_arr, locations_arr, arclength)
        result.append(
            api.models.SetIO(io=write.to_api_model(), location=location, io_origin=write.origin)
        )
    return result


def _write_triggers(
    combined_actions: CombinedActions,
) -> list[tuple[WriteAction, AtTrigger | None]]:
    return [(item, item.at) for item in combined_actions.items if isinstance(item, WriteAction)]


def _cumulative_arclength(positions: Sequence[Sequence[float]]) -> np.ndarray:
    """Cumulative Cartesian distance [mm] along the per-sample TCP positions."""
    pts = np.asarray(positions, dtype=float)
    if pts.ndim != 2 or len(pts) == 0:
        return np.zeros(len(pts))
    step = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(step)])


def _interp(x: float, xp: np.ndarray, fp: np.ndarray) -> float:
    """Linear interpolation of ``fp`` at ``x`` over a non-decreasing ``xp``.

    ``np.interp`` clamps to the endpoints outside ``[xp[0], xp[-1]]`` and copes with
    flat (repeated) ``xp`` regions, which both occur in planned trajectories (e.g. a
    wait holds a constant location).
    """
    return float(np.interp(x, xp, fp))


def _resolve(
    trigger: AtTrigger,
    anchor: int,
    n_motions: int,
    times: np.ndarray,
    locations: np.ndarray,
    arclength: np.ndarray | None,
) -> float:
    """Resolve one trigger anchored at motion boundary ``anchor`` to a float location."""
    if anchor >= n_motions:
        logger.warning(
            "Path trigger %r is placed after the last motion and has no following segment; "
            "it collapses to the trajectory end at location %s.",
            trigger,
            anchor,
        )
        return float(anchor)

    lower, upper = float(anchor), float(anchor + 1)

    if isinstance(trigger, PathFractionTrigger):
        return lower + trigger.value

    if isinstance(trigger, TimeTrigger):
        return _resolve_relative(
            times, locations, lower, upper, trigger.seconds, trigger.reference, trigger
        )

    if isinstance(trigger, DistanceTrigger):
        if arclength is None:
            logger.warning(
                "Distance trigger %r could not be resolved (no TCP positions available); "
                "falling back to the motion boundary at location %s.",
                trigger,
                anchor,
            )
            return lower
        return _resolve_relative(
            arclength, locations, lower, upper, trigger.millimeters, trigger.reference, trigger
        )

    raise TypeError(f"Unsupported path trigger: {trigger!r}")


def _resolve_relative(
    domain: np.ndarray,
    locations: np.ndarray,
    lower: float,
    upper: float,
    offset: float,
    reference: AtReference,
    trigger: AtTrigger,
) -> float:
    """Map an offset measured in ``domain`` (time or arc length) to a location.

    ``domain`` is the planned quantity the offset is measured in, parallel to
    ``locations``. ``PREVIOUS`` measures forward from the segment start, ``NEXT``
    backward from its end. Offsets that leave the segment are clamped to its boundary
    and a warning is logged.

    The lookup is restricted to the samples of the anchor segment: ``domain`` is only
    non-decreasing (arc length does not grow during a pure reorientation, time does
    not grow between identical samples), so a flat run spanning a segment boundary
    would otherwise let the reverse lookup land in a neighbouring segment.
    """
    in_segment = (locations >= lower) & (locations <= upper)
    segment_domain = domain[in_segment]
    segment_locations = locations[in_segment]
    if len(segment_domain) < 2:
        # Too coarsely sampled to interpolate inside the segment; the best we can do
        # is the boundary the trigger is measured from.
        return lower if reference is AtReference.PREVIOUS else upper

    domain_lower = float(segment_domain[0])
    domain_upper = float(segment_domain[-1])
    if reference is AtReference.PREVIOUS:
        target = domain_lower + offset
    else:
        target = domain_upper - offset
    clamped = float(np.clip(target, domain_lower, domain_upper))
    if not np.isclose(clamped, target):
        logger.warning(
            "Path trigger %r resolves outside its motion segment [%.1f, %.1f]; "
            "clamping to the segment boundary.",
            trigger,
            lower,
            upper,
        )
    if np.isclose(domain_lower, domain_upper):
        # Zero-extent segment (e.g. a pure reorientation for a distance trigger):
        # every offset collapses onto the boundary it is measured from.
        return lower if reference is AtReference.PREVIOUS else upper
    resolved = _interp(clamped, segment_domain, segment_locations)
    return float(np.clip(resolved, lower, upper))
