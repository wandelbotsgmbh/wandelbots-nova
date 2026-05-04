"""Feature mapping between LeRobot flat feature dicts and NOVA motion groups."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from nova.cell.motion_group import MotionGroup
    from nova.types import RobotState


@dataclass
class FeatureGroup:
    """Maps a named role to a motion group.

    The role defines the flat feature names the policy sees:
    - ``left`` -> ``left_joint_1.pos``, ``left_joint_2.pos``, ..., ``left_gripper.pos``
    - ``right`` -> ``right_joint_1.pos``, ``right_joint_2.pos``, ..., ``right_gripper.pos``

    That keeps the policy-side naming semantic and hardware-agnostic.
    """

    motion_group: MotionGroup
    """The NOVA motion group this feature group controls."""

    role: str
    """Semantic role used to derive feature names, e.g. ``left`` or ``right``."""

    num_joints: int | None = None
    """Number of joints. Defaults to the observed joint count if omitted."""

    gripper_io: str | None = None
    """NOVA IO key for the gripper output, e.g. ``digital_out[0]``. None = no gripper."""

    gripper_threshold: float = 50.0
    """Policy gripper value above this maps to a closed gripper (``True``)."""

    def joint_key(self, joint_index: int) -> str:
        """Return the flat feature key for a 1-based joint index."""
        return f"{self.role}_joint_{joint_index}.pos"

    @property
    def gripper_key(self) -> str:
        """Return the flat feature key for the gripper."""
        return f"{self.role}_gripper.pos"


@dataclass
class FeatureMap:
    """Maps between flat feature dicts and NOVA motion groups."""

    groups: list[FeatureGroup]

    def get_motion_groups(self) -> list[MotionGroup]:
        """Get all motion groups in order."""
        return [group.motion_group for group in self.groups]

    async def build_observation(self, states: dict[str, RobotState]) -> dict[str, float]:
        """Convert per-group RobotState into a flat feature dict."""
        obs: dict[str, float] = {}
        for group in self.groups:
            state = states.get(group.motion_group.id)
            if state is None:
                continue

            for joint_index, joint_value in enumerate(state.joints, start=1):
                obs[group.joint_key(joint_index)] = joint_value

            if group.gripper_io is None:
                continue

            try:
                io_values = await group.motion_group._api_client.controller_ios_api.list_io_values(
                    cell=group.motion_group._cell,
                    controller=group.motion_group._controller_id,
                    ios=[group.gripper_io],
                )
                closed = bool(io_values[0].root.value) if io_values else False
                obs[group.gripper_key] = 100.0 if closed else 0.0
            except Exception:
                obs[group.gripper_key] = 0.0

        return obs

    def parse_action(self, action: dict[str, float]) -> tuple[
        dict[str, list[list[float]]],
        dict[str, dict[str, bool | int | float | str]] | None,
    ]:
        """Convert a flat action dict into per-group joints and IOs."""
        joints: dict[str, list[list[float]]] = {}
        ios: dict[str, dict[str, bool | int | float | str]] = {}

        for group in self.groups:
            motion_group_id = group.motion_group.id
            joint_count = group.num_joints or 6

            joint_values: list[float] = []
            for joint_index in range(1, joint_count + 1):
                key = group.joint_key(joint_index)
                if key in action:
                    joint_values.append(float(action[key]))

            if joint_values:
                joints[motion_group_id] = [joint_values]

            if group.gripper_io is None or group.gripper_key not in action:
                continue

            gripper_value = float(action[group.gripper_key])
            ios[motion_group_id] = {
                group.gripper_io: gripper_value >= group.gripper_threshold,
            }

        return joints, ios or None
