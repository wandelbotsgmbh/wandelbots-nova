"""Software limits on TCP targets, as a stop condition.

``await TcpLimits(...).stop_condition(motion_group)`` returns an ordinary stop condition:
the executor holds every chunk (the steps that will run) against it before sending, and a
chunk that breaks a limit ends the run normally, naming the step and the limit. TCP actions
are checked directly; joint actions along the TCP path forward kinematics gives for
``tcp``. This is a check in the executor's own path, not a safety system: safety belongs on
the robot controller.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import TYPE_CHECKING, cast

from scipy.spatial.transform import Rotation

from novapolicy import poses
from novapolicy.kinematics import Kinematics

if TYPE_CHECKING:
    import numpy as np

    from nova.cell.motion_group import MotionGroup
    from novapolicy.types import StopContext

_FK_TOLERANCE_MM = 1.0  # forward kinematics vs. NOVA's reported TCP pose
_FK_TOLERANCE_DEG = 0.5


@dataclass(frozen=True, slots=True)
class TcpLimits:
    """Limits on the TCP targets of every chunk. ``None`` disables a limit."""

    max_speed_mm_s: float | None = None
    """TCP distance between consecutive targets, as a speed at the chunk's step spacing."""
    max_rotation_speed_deg_s: float | None = None
    """Tool rotation between consecutive targets, as a speed at the chunk's step spacing."""
    max_chunk_travel_mm: float | None = None
    """Distance of any target from the TCP position at inference time."""
    x_mm: tuple[float, float] | None = None
    """Allowed ``(min, max)`` of target x in ``coordinate_system``; likewise y and z."""
    y_mm: tuple[float, float] | None = None
    z_mm: tuple[float, float] | None = None
    coordinate_system: str = ""
    """The coordinate system of the controller the bounds are in; ``""`` is the world."""

    def __post_init__(self) -> None:
        for name in ("x_mm", "y_mm", "z_mm"):
            bounds = getattr(self, name)
            if bounds is not None and not bounds[0] < bounds[1]:
                raise ValueError(f"TcpLimits.{name} must be (min, max) with min < max")

    async def stop_condition(
        self, motion_group: MotionGroup, *, tcp: str | None = None
    ) -> TcpLimitsCondition:
        """A stop condition holding ``motion_group``'s chunks against these limits.

        ``tcp``: for joint actions, the TCP whose path is checked; its forward kinematics
        come from the motion group's description and must agree with the pose NOVA reports
        for the current joints. Without it, joint actions are not checked.
        """
        frame = (
            await poses.coordinate_system_in_world(motion_group, self.coordinate_system)
            if self.coordinate_system
            else None
        )
        kinematics = None
        if tcp is not None:
            kinematics = Kinematics.from_description(await motion_group.get_description(), tcp)
            state = await motion_group.get_state(tcp)
            computed = poses.matrix_pose(kinematics.fk(list(state.joints)))
            reported = [*state.pose.position, *state.pose.orientation]
            offset_mm = math.dist(computed[:3], reported[:3])
            turn_deg = _angle_deg(computed[3:], reported[3:])
            if offset_mm > _FK_TOLERANCE_MM or turn_deg > _FK_TOLERANCE_DEG:
                msg = (
                    f"Forward kinematics of {motion_group.id!r} disagree with NOVA at the "
                    f"current joints ({offset_mm:.1f} mm, {turn_deg:.2f} deg): cannot check "
                    "TcpLimits on its joint actions"
                )
                raise ValueError(msg)
        return TcpLimitsCondition(self, motion_group.id, frame=frame, kinematics=kinematics)


class TcpLimitsCondition:
    """A stop condition for :class:`TcpLimits`; build it with ``TcpLimits.stop_condition``.

    It fires on a chunk about to be sent and names the broken limit in ``__name__``, which
    the run's ``result.reason`` reports; ``violations`` holds all of them.
    """

    def __init__(
        self,
        limits: TcpLimits,
        motion_group_id: str,
        *,
        frame: np.ndarray | None = None,
        kinematics: Kinematics | None = None,
    ) -> None:
        self.limits = limits
        self.motion_group_id = motion_group_id
        self._frame = frame
        self._kinematics = kinematics
        self.violations: list[str] = []
        self.__name__ = "tcp_limits"

    def __call__(self, ctx: StopContext) -> bool:
        if ctx.motion_group_id != self.motion_group_id or ctx.target_dt_ms is None:
            return False  # not this robot, or a jogging tick rather than a chunk
        if ctx.target_tcp:
            if ctx.state.pose is None:
                return False
            start = [*ctx.state.pose.position, *ctx.state.pose.orientation]
            path = ctx.target_tcp
        elif ctx.target_joints and self._kinematics is not None:
            start = poses.matrix_pose(self._kinematics.fk(list(ctx.state.joints)))
            path = [poses.matrix_pose(self._kinematics.fk(q)) for q in ctx.target_joints]
        else:
            return False
        start = poses.world_to_frame(start, self._frame)
        path = [poses.world_to_frame(p, self._frame) for p in path]
        self.violations = tcp_limit_violations(start, path, ctx.target_dt_ms, self.limits)
        if self.violations:
            self.__name__ = f"tcp_limits ({self.violations[0]})"
            return True
        return False


def _angle_deg(rotvec_a: list[float], rotvec_b: list[float]) -> float:
    turn = cast("Rotation", Rotation.from_rotvec(rotvec_a) * Rotation.from_rotvec(rotvec_b).inv())
    return math.degrees(turn.magnitude())


def tcp_limit_violations(
    start: list[float], targets: list[list[float]], dt_ms: float, limits: TcpLimits
) -> list[str]:
    """Every limit ``targets`` break when the TCP starts at ``start``; empty when none."""
    broken: list[str] = []
    if dt_ms <= 0 and (limits.max_speed_mm_s is not None or limits.max_rotation_speed_deg_s):
        broken.append(f"dt_ms {dt_ms}: speed limits need the chunk's step spacing")
    prev = start
    for i, t in enumerate(targets):
        dt_s = dt_ms / 1000.0 if dt_ms > 0 else float("inf")
        speed = math.dist(prev[:3], t[:3]) / dt_s
        turn = cast(
            "Rotation", Rotation.from_rotvec(t[3:6]) * Rotation.from_rotvec(prev[3:6]).inv()
        )
        rot_speed = math.degrees(turn.magnitude()) / dt_s
        travel = math.dist(start[:3], t[:3])
        checks = [
            (limits.max_speed_mm_s, speed, f"speed {speed:.0f} > {limits.max_speed_mm_s} mm/s"),
            (
                limits.max_rotation_speed_deg_s,
                rot_speed,
                f"rotation {rot_speed:.0f} > {limits.max_rotation_speed_deg_s} deg/s",
            ),
            (
                limits.max_chunk_travel_mm,
                travel,
                f"travel {travel:.0f} > {limits.max_chunk_travel_mm} mm",
            ),
        ]
        broken += [
            f"step {i}: {text}"
            for limit, value, text in checks
            if limit is not None and value > limit
        ]
        for axis, bounds in enumerate((limits.x_mm, limits.y_mm, limits.z_mm)):
            if bounds is not None and not bounds[0] <= t[axis] <= bounds[1]:
                broken.append(f"step {i}: {'xyz'[axis]} {t[axis]:.0f} outside {bounds} mm")
        prev = t
    return broken
