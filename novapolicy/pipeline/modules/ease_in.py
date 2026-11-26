"""``EaseIn``: blend from where the robot stood (docs/jogging.md)."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from novapolicy.pipeline.base import ModuleContext, TimedWaypoint


@dataclass
class EaseIn:
    """Waypoint module: blend from where the robot stood over the first ``duration_s``.

    A smootherstep of each waypoint's own timestamp since the first one (docs/jogging.md); TCP
    rotation vectors blend per component.
    """

    duration_s: float
    _start: tuple[int, list[float]] | None = field(default=None, init=False, repr=False)

    def __post_init__(self) -> None:
        if self.duration_s <= 0:
            raise ValueError("EaseIn needs a positive duration_s")

    def on_waypoints(
        self, waypoints: list[TimedWaypoint], context: ModuleContext
    ) -> list[list[float]]:
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
