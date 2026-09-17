"""Path triggers ("Bahnschaltpunkte") for positioning IO writes within a motion.

A path trigger attaches an :func:`~nova.actions.io.io_write` to a precise point on the
planned path *within* a motion, instead of only at the integer motion-command
boundaries.

Place the write directly *before* the motion it belongs to. Without a trigger it fires
at that boundary, i.e. when the previous motion has finished and the upcoming motion
starts. A trigger moves it into the upcoming motion, measured either from the
motion's **start** or back from its **target**. The trigger can never leave that
motion: a write placed in one part of the program never fires somewhere else on the
path.

Use the builders::

    io_write("relay", True, at=after_start(seconds=0.3))      # 0.3 s after the motion starts
    io_write("relay", True, at=after_start(millimeters=50))   # 50 mm of TCP travel after start
    io_write("relay", True, at=before_target(seconds=0.3))    # 0.3 s before reaching the target
    io_write("relay", True, at=before_target(millimeters=50)) # 50 mm of TCP travel before target
    io_write("relay", True, at=at_path_fraction(0.5))         # halfway through the motion

``seconds`` also accepts a :class:`datetime.timedelta`.

The trigger objects are the ones the NOVA command-routine API uses for its ``at`` field
(``api.models.AtTrigger``), so an action list and a command routine share one
vocabulary:

- :class:`~nova.api.models.TimeTrigger` / :class:`~nova.api.models.DistanceTrigger` with
  ``reference=PREVIOUS`` (measured from the start of the upcoming motion, i.e. the end
  of the previous one) or ``reference=NEXT`` (measured back from the target of the
  upcoming motion). ``after_start`` / ``before_target`` fix the reference for you; the
  explicit form is :func:`nova.command_routines.at_time` / ``at_distance``.
- :class:`~nova.api.models.PathFractionTrigger` — a fraction ``[0, 1)`` of the upcoming
  motion (``0.0`` = start, ``0.5`` = halfway to the target). ``1.0`` is "at the
  target", which you express by placing the write *after* the motion without a trigger.

Time and distance triggers are resolved against the planned trajectory when it is
executed (time against the planned time profile, distance against the cumulative TCP
path length obtained via forward kinematics). Offsets that would leave the motion are
clamped to its boundary and a warning is logged. A trigger placed after the last motion
has no motion to move into and collapses to the trajectory end. See
:mod:`nova.actions.path_trigger_resolver`.
"""

from __future__ import annotations

from datetime import timedelta

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
    "after_start",
    "at_path_fraction",
    "before_target",
]


def after_start(
    *, seconds: float | timedelta | None = None, millimeters: float | None = None
) -> TimeTrigger | DistanceTrigger:
    """Trigger ``seconds`` or ``millimeters`` of TCP travel after the upcoming motion starts.

    Pass exactly one of the two keywords.
    """
    return _relative_trigger(AtReference.PREVIOUS, seconds, millimeters)


def before_target(
    *, seconds: float | timedelta | None = None, millimeters: float | None = None
) -> TimeTrigger | DistanceTrigger:
    """Trigger ``seconds`` or ``millimeters`` of TCP travel before the upcoming motion
    reaches its target.

    Pass exactly one of the two keywords.
    """
    return _relative_trigger(AtReference.NEXT, seconds, millimeters)


def _relative_trigger(
    reference: api.models.AtReference, seconds: float | timedelta | None, millimeters: float | None
) -> TimeTrigger | DistanceTrigger:
    if (seconds is None) == (millimeters is None):
        raise ValueError("pass exactly one of seconds= or millimeters=")
    if seconds is not None:
        if isinstance(seconds, timedelta):
            seconds = seconds.total_seconds()
        return at_time(seconds, reference)
    assert millimeters is not None
    return at_distance(millimeters, reference)
