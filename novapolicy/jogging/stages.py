"""Opt-in stages that reshape waypoint values right before they are sent to NOVA.

The executor and the joggers map and transport; nothing in them shapes motion. A stage is the one
place where a caller can still do so on purpose, for every path alike: ``PolicyExecutor``,
``jog_joints`` and ``jog_tcp`` take ``stages=[...]`` and hand them to their sessions.

Contract:

* A stage runs in the session after placement, on every request: it receives the waypoints about
  to be sent as ``(session-clock timestamp, values)`` and returns their new values, one per
  waypoint. It cannot move, add or drop a waypoint, so every timing contract holds unchanged
  (IO at the first step, the sequential settle, a queue's timeline, the drain).
* It always receives the raw values, also for waypoints it saw before (the rolling window re-sends
  them), so it must give the same timestamp the same result: a pure function of the timestamp, the
  raw values and state it fixed on its first call.
* Each session runs its own copy of every stage, so two arms never share state.
* Stages run in the order given. No stage runs unless one is passed.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Protocol

if TYPE_CHECKING:
    from novapolicy.jogging.clock import ServerMs

TimedWaypoint = tuple["ServerMs", list[float]]
"""A waypoint about to be sent: its time on the session clock and its values (joints in rad, or
a TCP pose ``[x, y, z, rx, ry, rz]`` in mm and rad)."""


@dataclass(frozen=True, slots=True)
class StageContext:
    """What a stage may read besides the waypoints."""

    now_ms: ServerMs
    """The session clock now."""
    position: list[float] | None
    """The robot's latest measured position, in the waypoints' form; ``None`` before the first
    state arrived."""


class WaypointStage(Protocol):
    """Reshapes the values of the waypoints about to be sent; see the module contract."""

    def __call__(self, waypoints: list[TimedWaypoint], context: StageContext) -> list[list[float]]:
        """The new values, one per waypoint, in order."""
        ...


@dataclass
class EaseIn:
    """Blend the commanded values in from where the robot stood, over the first ``duration_s``.

    The blend is a smootherstep (``6x^5 - 15x^4 + 10x^3``) of the time since the first waypoint
    this stage saw, so velocity and acceleration start and end at zero; a straight ramp steps the
    velocity when it completes, and a smoothstep the acceleration. Each waypoint is blended by its
    own timestamp, so overlapping chunks agree on every moment. TCP poses blend their rotation
    vector component by component, which is fine for the small rotations at a start.
    """

    duration_s: float
    _start: tuple[int, list[float]] | None = field(default=None, init=False, repr=False)

    def __post_init__(self) -> None:
        if self.duration_s <= 0:
            raise ValueError("EaseIn needs a positive duration_s")

    def __call__(self, waypoints: list[TimedWaypoint], context: StageContext) -> list[list[float]]:
        values = [list(v) for _, v in waypoints]
        if not waypoints:
            return values
        if self._start is None:
            if context.position is None:
                return values  # nothing measured yet to blend from
            self._start = (waypoints[0][0], list(context.position))
        t0_ms, start = self._start
        blended: list[list[float]] = []
        for (t_ms, _), value in zip(waypoints, values, strict=True):
            x = min(1.0, max(0.0, (t_ms - t0_ms) / (1000.0 * self.duration_s)))
            if x >= 1.0:
                blended.append(value)
                continue
            e = x**3 * (x * (x * 6.0 - 15.0) + 10.0)
            blended.append([s + e * (v - s) for s, v in zip(start, value, strict=True)])
        return blended
