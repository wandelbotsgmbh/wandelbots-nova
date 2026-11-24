"""Chunk module: follow live targets with a spring-damper and send its braking path (ADR 0022)."""

from __future__ import annotations

from dataclasses import dataclass, field, replace
import math
from typing import TYPE_CHECKING, ClassVar

from novapolicy.transport.clock import ServerMs

if TYPE_CHECKING:
    from novapolicy.pipeline.base import ModuleContext
    from novapolicy.transport.waypoints import PendingChunk

TICK_MS = 2.0
"""The integration step."""
CYCLE_MS = 8
"""NOVA's controller cycle: the first waypoint is due on one (ADR 0021)."""
_NO_ROTATION = 1e-9


@dataclass
class ServoHorizon:
    """Chunk module: a live target becomes the reference of a spring-damper,
    ``q'' = gain * (ref - (q + lookahead_s * q'))``, and the chunk sent is that spring's path
    with the target held: where it is when the target is due, then braking onto it.

    A live target says where the hand is now; sent alone, NOVA's follower drives at it without
    braking for what comes next and overshoots a sudden stop. The spring rounds a turn the arm
    cannot take into one it can, holds a constant speed ``lookahead_s`` behind the target and, at
    ``lookahead_s = 2 / sqrt(gain)``, settles without overshoot. Accelerations and speeds above the
    limits scale the whole step down, keeping its direction. The waypoints are
    ``step_ms`` apart for ``horizon_ms``, the first on NOVA's 8 ms cycle (ADR 0021).

    For live targets (``set_target``) only: a chunk of several steps, or one without its own
    start time, passes through and resets the state. TCP poses are ``[x, y, z, rx, ry, rz]`` in
    mm and rad; a rotation vector is taken as the equivalent closest to the spring's, so a tool
    turning through a half-turn does not swing the long way. Joint targets use the angular limits
    per joint.
    """

    stage: ClassVar[str] = "input"  # its place among the chunk modules (CHUNK_STAGES)

    gain: float = 300.0
    """Spring stiffness, 1/s²."""
    lookahead_s: float = 0.115
    """Damping time, s: the lag behind a target at constant speed."""
    horizon_ms: float = 400.0
    step_ms: float = 32.0
    linear_acceleration: float = 8000.0
    """mm/s², TCP position."""
    linear_speed: float = 1500.0
    """mm/s, TCP position."""
    angular_acceleration: float = 40.0
    """rad/s², TCP rotation or each joint."""
    angular_speed: float = 3.0
    """rad/s, TCP rotation or each joint."""
    _t_ms: float | None = field(default=None, init=False, repr=False)
    _q: list[float] = field(default_factory=list, init=False, repr=False)
    _qd: list[float] = field(default_factory=list, init=False, repr=False)

    def __post_init__(self) -> None:
        positive = (
            self.gain,
            self.lookahead_s,
            self.horizon_ms,
            self.step_ms,
            self.linear_acceleration,
            self.linear_speed,
            self.angular_acceleration,
            self.angular_speed,
        )
        if not all(math.isfinite(v) and v > 0 for v in positive):
            raise ValueError("ServoHorizon needs positive finite settings")
        if self.step_ms < TICK_MS or self.horizon_ms < self.step_ms:
            raise ValueError("ServoHorizon needs step_ms >= 2 and horizon_ms >= step_ms")

    def on_chunk(self, chunk: PendingChunk, context: ModuleContext) -> PendingChunk:
        if len(chunk.steps) != 1 or chunk.first_timestamp_ms is None:
            self._t_ms = None
            return chunk
        cartesian = context.mode == "cartesian"
        ref = list(chunk.steps[0])
        start_ms = float(math.ceil(chunk.first_timestamp_ms / CYCLE_MS) * CYCLE_MS)
        if self._t_ms is None or len(self._q) != len(ref):
            self._t_ms, self._q, self._qd = start_ms, list(ref), [0.0] * len(ref)
        if cartesian:
            ref[3:6] = _closest_rotation_vector(ref[3:6], self._q[3:6])
        groups = self._groups(context.mode, len(ref))
        while self._t_ms + 1e-9 < start_ms:  # the spring until the target is due
            self._q, self._qd = self._tick(self._q, self._qd, ref, groups)
            self._t_ms += TICK_MS
        steps, q, qd = [list(self._q)], list(self._q), list(self._qd)
        every = round(self.step_ms / TICK_MS)
        for tick in range(1, round(self.horizon_ms / TICK_MS) + 1):  # then the target held
            q, qd = self._tick(q, qd, ref, groups)
            if tick % every == 0:
                steps.append(list(q))
        return replace(
            chunk,
            steps=steps,
            dt_ms=every * TICK_MS,
            first_timestamp_ms=ServerMs(round(self._t_ms)),
            timestamp_offset_steps=0,
            lead=False,
            observed_at_ms=None,
            extend_buffer=False,
            caller_step_count=len(steps),
            # The spring's position now is the first step: the target's IO and hooks go with it.
            when_due=None if chunk.when_due is None else (0, chunk.when_due[1]),
        )

    def _groups(self, mode: str, width: int) -> list[tuple[range, float, float]]:
        """The axes limited together: TCP position, TCP rotation, or each joint."""
        if mode == "cartesian":
            return [
                (range(3), self.linear_acceleration, self.linear_speed),
                (range(3, 6), self.angular_acceleration, self.angular_speed),
            ]
        return [
            (range(i, i + 1), self.angular_acceleration, self.angular_speed) for i in range(width)
        ]

    def _tick(
        self,
        q: list[float],
        qd: list[float],
        ref: list[float],
        groups: list[tuple[range, float, float]],
    ) -> tuple[list[float], list[float]]:
        """One tick: the spring, one scale for all axes, semi-implicit Euler."""
        dt = TICK_MS / 1000.0
        qdd = [
            self.gain * (r - (x + self.lookahead_s * v)) for r, x, v in zip(ref, q, qd, strict=True)
        ]
        scale = 1.0
        for axes, max_acceleration, max_speed in groups:
            acceleration = math.hypot(*(qdd[i] for i in axes))
            if acceleration > max_acceleration:
                scale = min(scale, max_acceleration / acceleration)
            speed = math.hypot(*(qd[i] for i in axes))
            next_speed = math.hypot(*(qd[i] + qdd[i] * dt for i in axes))
            if next_speed > max_speed and acceleration > 0:
                scale = min(scale, max(0.0, (max_speed - speed) / (acceleration * dt)))
        qd = [v + a * scale * dt for v, a in zip(qd, qdd, strict=True)]
        return [x + v * dt for x, v in zip(q, qd, strict=True)], qd


def _closest_rotation_vector(rotvec: list[float], near: list[float]) -> list[float]:
    """``rotvec``, or the same rotation the other way round the axis, whichever is closer to
    ``near``: past a half-turn the shortest vector flips sign, a jump of 2 pi."""
    angle = math.hypot(*rotvec)
    if angle < _NO_ROTATION:
        return list(rotvec)
    other = [v * (1.0 - 2.0 * math.pi / angle) for v in rotvec]
    if math.dist(other, near) < math.dist(rotvec, near):
        return other
    return list(rotvec)
