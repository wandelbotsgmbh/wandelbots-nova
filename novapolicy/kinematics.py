"""Forward kinematics of a NOVA motion group, from its own description.

Uses the DH parameters, the TCP offset and the mounting that NOVA reports for the motion
group (``MotionGroup.get_description()``), so a TCP pose can be computed for joint targets
without a round trip to the controller.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

from novapolicy.poses import pose_matrix

if TYPE_CHECKING:
    from collections.abc import Iterable, Sequence

    from nova import api


def _pose_or_identity(
    position: Iterable[float] | None, rotvec: Iterable[float] | None
) -> np.ndarray:
    """``pose_matrix`` with missing parts of an API pose taken as zero."""
    return pose_matrix([*(position or (0.0, 0.0, 0.0)), *(rotvec or (0.0, 0.0, 0.0))])


@dataclass
class Kinematics:
    """Forward kinematics of a motion group with one of its TCPs (mm, rad)."""

    dh: list[tuple[float, float, float, float, bool]]  # a, alpha, d, theta offset, reversed
    tcp: np.ndarray  # flange -> TCP
    mounting: np.ndarray  # world -> robot base

    @classmethod
    def from_description(
        cls, description: api.models.MotionGroupDescription, tcp_name: str
    ) -> Kinematics:
        """From ``MotionGroup.get_description()`` and the name of one of its TCPs."""
        dh = [
            (
                p.a or 0.0,
                p.alpha or 0.0,
                p.d or 0.0,
                p.theta or 0.0,
                bool(p.reverse_rotation_direction),
            )
            for p in description.dh_parameters or []
        ]
        tcps = description.tcps or {}
        if tcp_name not in tcps:
            msg = f"TCP {tcp_name!r} is not defined on this motion group ({sorted(tcps)})"
            raise ValueError(msg)
        tcp_pose = tcps[tcp_name].pose
        mount = description.mounting
        mounting = _pose_or_identity(mount.position, mount.orientation) if mount else np.eye(4)
        return cls(dh, _pose_or_identity(tcp_pose.position, tcp_pose.orientation), mounting)

    def fk(self, joints: Sequence[float]) -> np.ndarray:
        """TCP pose (4x4, mm) in the world for the given joint positions (rad)."""
        t = self.mounting.copy()
        for (a, alpha, d, theta, reverse), q in zip(self.dh, joints, strict=True):
            th = theta + (-q if reverse else q)
            ca, sa, ct, st = np.cos(alpha), np.sin(alpha), np.cos(th), np.sin(th)
            t @= np.array([
                [ct, -st * ca, st * sa, a * ct],
                [st, ct * ca, -ct * sa, a * st],
                [0, sa, ca, d],
                [0, 0, 0, 1],
            ])
        return t @ self.tcp
