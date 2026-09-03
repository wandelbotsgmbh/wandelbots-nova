"""Path triggers ("Bahnschaltpunkte") for positioning IO writes between motions.

A path trigger attaches an :func:`~nova.actions.io.io_write` to a precise point on the
planned path *between* two motion actions, instead of only at the integer
motion-command boundaries.

Every trigger is *anchored*: the write's position in the action list selects the
motion segment it belongs to (the segment between the previous and the next motion
action), and the trigger only addresses a point *within* that segment. A trigger
placed in one part of the program can therefore never fire somewhere completely
different on the path.

The trigger types are the ones the NOVA command-routine API uses for its ``at``
field (``api.models.AtTrigger``), so an action list and a command routine share one
vocabulary:

- :class:`~nova.api.models.PathFractionTrigger` — a fraction ``[0, 1)`` within the
  anchor segment (``0.0`` = at the previous motion, ``0.5`` = halfway to the next).
- :class:`~nova.api.models.TimeTrigger` — seconds measured from the previous motion
  (``reference=PREVIOUS``) or back from the next motion (``reference=NEXT``).
- :class:`~nova.api.models.DistanceTrigger` — Cartesian TCP millimeters measured from
  the previous motion or back from the next motion.

Time and distance triggers are resolved against the planned trajectory when the
trajectory is executed (time against the planned time profile, distance against the
cumulative TCP path length obtained via forward kinematics). Offsets that would leave
the anchor segment are clamped to the segment boundary and a warning is logged. A
trigger placed after the last motion has no following segment and collapses to the
trajectory end. See :mod:`nova.actions.path_trigger_resolver`.

Use the builders rather than the model classes directly::

    io_write("relay", True, at=at_path_fraction(0.5))  # halfway through the anchor segment
    io_write("relay", True, at=after_time(0.5))        # 0.5 s after the previous motion
    io_write("relay", True, at=before_time(0.5))       # 0.5 s before the next motion
    io_write("relay", True, at=after_distance(100))    # 100 mm after the previous motion
    io_write("relay", True, at=before_distance(50))    # 50 mm before the next motion

``at_path_fraction``, ``at_distance`` and ``at_time`` are the same builders that
:mod:`nova.command_routines` uses; ``after_*`` / ``before_*`` are shorthands that
fix the ``reference``.
"""

from __future__ import annotations

from nova import api
from nova.command_routines.commands import at_distance, at_path_fraction, at_time

AtReference = api.models.AtReference
AtTrigger = api.models.AtTrigger
DistanceTrigger = api.models.DistanceTrigger
PathFractionTrigger = api.models.PathFractionTrigger
TimeTrigger = api.models.TimeTrigger

__all__ = [
    "AtReference",
    "AtTrigger",
    "DistanceTrigger",
    "PathFractionTrigger",
    "TimeTrigger",
    "at_distance",
    "at_path_fraction",
    "at_time",
    "after_distance",
    "after_time",
    "before_distance",
    "before_time",
]


def after_time(seconds: float) -> TimeTrigger:
    """Trigger ``seconds`` after the previous motion action."""
    return at_time(seconds, AtReference.PREVIOUS)


def before_time(seconds: float) -> TimeTrigger:
    """Trigger ``seconds`` before the next motion action."""
    return at_time(seconds, AtReference.NEXT)


def after_distance(millimeters: float) -> DistanceTrigger:
    """Trigger ``millimeters`` of TCP travel after the previous motion action."""
    return at_distance(millimeters, AtReference.PREVIOUS)


def before_distance(millimeters: float) -> DistanceTrigger:
    """Trigger ``millimeters`` of TCP travel before the next motion action."""
    return at_distance(millimeters, AtReference.NEXT)
