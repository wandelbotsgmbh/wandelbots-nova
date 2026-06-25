"""Path triggers ("Bahnschaltpunkte") for positioning IO events between motions.

A :class:`PathTrigger` lets a user attach an IO write (or, in the future, other
events) to a precise point on the planned path *between* two motion actions,
instead of only at the integer motion-command boundaries.

All three addressing modes are *anchored*: the trigger's position in the action
list selects the motion segment it belongs to (the segment between the previous
and the next motion action), and the trigger value only addresses a point
*within* that segment. This avoids the foot-gun of a trigger placed in one part
of the program firing somewhere completely different on the path.

Three addressing modes are supported:

- :class:`PathParameterTrigger` — a fraction within the anchor segment
  (``0.0`` = at the previous motion, ``1.0`` = at the next motion). No planned
  trajectory is needed to resolve it.
- :class:`TimeTrigger` — a duration in seconds measured from the previous motion
  (``reference=PREVIOUS``) or back from the next motion (``reference=NEXT``).
- :class:`DistanceTrigger` — a Cartesian TCP distance in millimeters measured
  from the previous motion or back from the next motion.

Values that would land outside the anchor segment are clamped to the segment
boundary (and a warning is logged).

Time- and distance-based triggers can only be turned into a concrete path
location once the trajectory is planned (they are resolved against the planned
trajectory's per-sample ``times`` / ``locations`` and, for distance, the TCP
poses). See :mod:`nova.actions.path_trigger_resolver`.

Use the convenience constructors rather than the model classes directly::

    io_write("relay", True, at=at_path(0.5))          # halfway through the anchor segment
    io_write("relay", True, at=after_time(0.5))        # 0.5 s after the previous motion
    io_write("relay", True, at=before_time(0.5))       # 0.5 s before the next motion
    io_write("relay", True, at=after_distance(100))    # 100 mm after the previous motion
    io_write("relay", True, at=before_distance(50))    # 50 mm before the next motion
"""

from __future__ import annotations

from enum import StrEnum
from typing import Annotated, Literal, Union

import pydantic


class TriggerReference(StrEnum):
    """Which motion a relative (time/distance) trigger is measured against."""

    PREVIOUS = "previous"
    """Measured forward from the previous motion action."""

    NEXT = "next"
    """Measured backward from the next motion action."""


class PathParameterTrigger(pydantic.BaseModel):
    """Trigger at a fraction within the anchor motion segment.

    ``value`` is interpolation within the segment of the action's anchor:
    ``0.0`` fires at the previous motion, ``1.0`` at the next motion and
    ``0.5`` halfway between them. The anchor segment is determined by the
    position of the action in the action list, so usually only values in
    ``[0, 1]`` make sense; values outside that range are clamped to the
    segment boundary when resolved.
    """

    type: Literal["path_parameter"] = "path_parameter"
    value: float


class TimeTrigger(pydantic.BaseModel):
    """Trigger a fixed duration before/after the anchor motion.

    Resolved against the planned trajectory's time profile.
    """

    type: Literal["time"] = "time"
    seconds: float = pydantic.Field(ge=0)
    reference: TriggerReference = TriggerReference.PREVIOUS


class DistanceTrigger(pydantic.BaseModel):
    """Trigger a fixed Cartesian TCP distance before/after the anchor motion.

    Resolved against the cumulative TCP path length of the planned trajectory.
    """

    type: Literal["distance"] = "distance"
    millimeters: float = pydantic.Field(ge=0)
    reference: TriggerReference = TriggerReference.PREVIOUS


PathTrigger = Annotated[
    Union[PathParameterTrigger, TimeTrigger, DistanceTrigger], pydantic.Field(discriminator="type")
]
"""A path trigger in one of the supported addressing modes."""


def at_path(value: float) -> PathParameterTrigger:
    """Trigger at a fraction within the anchor segment (``0.0``..``1.0``).

    ``0.0`` fires at the previous motion, ``1.0`` at the next motion and ``0.5``
    halfway between them.
    """
    return PathParameterTrigger(value=value)


def after_time(seconds: float) -> TimeTrigger:
    """Trigger ``seconds`` after the previous motion action."""
    return TimeTrigger(seconds=seconds, reference=TriggerReference.PREVIOUS)


def before_time(seconds: float) -> TimeTrigger:
    """Trigger ``seconds`` before the next motion action."""
    return TimeTrigger(seconds=seconds, reference=TriggerReference.NEXT)


def after_distance(millimeters: float) -> DistanceTrigger:
    """Trigger ``millimeters`` of TCP travel after the previous motion action."""
    return DistanceTrigger(millimeters=millimeters, reference=TriggerReference.PREVIOUS)


def before_distance(millimeters: float) -> DistanceTrigger:
    """Trigger ``millimeters`` of TCP travel before the next motion action."""
    return DistanceTrigger(millimeters=millimeters, reference=TriggerReference.NEXT)
