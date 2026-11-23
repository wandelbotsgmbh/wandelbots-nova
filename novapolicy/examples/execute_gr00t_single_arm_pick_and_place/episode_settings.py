"""The settings this example's policy needs, measured once from one of its training episodes.

``episode_settings.json`` holds, per robot, what the episode's columns mean: the rate, the start
joints, whether actions are joint or TCP targets (absolute or deltas, and in which coordinate
system), the scale between action units and the motion the training robot realized, and how the
binary gripper is driven. :meth:`RobotSettings.action` turns that into the robot's ``Action``.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any

_JOINT_KINDS = ("joint_absolute", "joint_delta")


@dataclass(frozen=True)
class RobotSettings:
    """What one robot's columns of the episode mean."""

    motion_group: str
    tcp: str
    joints_key: str
    action_key: str
    action_kind: str  # joint_absolute | joint_delta | tcp_absolute | tcp_delta
    fit_r2: float  # how well the chosen reading explains the motion (position for TCP)
    # Realized motion per action unit: rad (joints) or mm (TCP position), rad (TCP rotation).
    scale: float
    rotation_scale: float | None  # TCP kinds only
    use_tool_coordinate_system: bool  # tcp_delta only; False: world
    home_deg: list[float]  # the episode's start joints
    gripper_state_key: str | None
    gripper_action_key: str | None
    gripper_close_when_negative: bool | None
    gripper_open: float | None
    gripper_closed: float | None
    gripper_holding: float | None

    def action(self, motion_group: Any) -> Any:
        """The ``Action`` entry for this robot's arm, with units and scale as measured."""
        from novapolicy import Action, Scale

        to_unit = [Scale(1.0 / self.scale)]  # NOVA unit -> policy unit
        if self.action_kind in _JOINT_KINDS:
            mode = "relative" if self.action_kind == "joint_delta" else "absolute"
            return Action.joint_positions(
                self.action_key, target=motion_group, mode=mode, ops=to_unit
            )
        assert self.rotation_scale is not None
        common: dict[str, Any] = {
            "position_ops": [Scale(0.001), Scale(1000.0 / self.scale)],  # mm -> m -> unit
            "orientation_ops": [Scale(1.0 / self.rotation_scale)],
        }
        if self.action_kind == "tcp_absolute":
            return Action.tcp(self.action_key, target=motion_group, **common)
        return Action.tcp(
            self.action_key,
            target=motion_group,
            mode="relative",
            accumulate=True,
            use_tool_coordinate_system=self.use_tool_coordinate_system,
            **common,
        )


@dataclass(frozen=True)
class EpisodeSettings:
    episode: str
    rate_hz: float
    robots: dict[str, RobotSettings]

    @classmethod
    def load(cls, path: str | Path) -> EpisodeSettings:
        raw = json.loads(Path(path).read_text(encoding="utf-8"))
        robots = {name: RobotSettings(**r) for name, r in raw["robots"].items()}
        return cls(episode=raw["episode"], rate_hz=raw["rate_hz"], robots=robots)
