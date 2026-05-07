"""Feature mapping between flat feature dicts and NOVA motion groups.

FeatureMap is a static, pure data structure: it declares robot topology and
translates between flat feature dicts and per-motion-group joints/IOs.

It has NO lifecycle (no start/stop, no streaming, no I/O). IO streaming
is managed by the executor.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from enum import StrEnum
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from nova.cell.motion_group import MotionGroup
    from nova.types import RobotState

from policy._sdk import get_controller_id
from policy.pose import pose_to_tcp

logger = logging.getLogger(__name__)


@dataclass
class GroupObservation:
    """Per-group extracted observation data.

    Produced by ``FeatureMap.build_grouped_observation()`` and consumed by
    policy clients to build their transport-specific formats.
    """

    group: FeatureGroup
    joints: list[float]
    tcp: list[float] | None = None
    ios: dict[str, float] | None = None


class TcpFormat(StrEnum):
    """TCP pose representation format for policy observations.

    Position is always in meters [x, y, z].
    """

    NONE = ""
    """Don't include TCP pose."""

    ROTATION_VECTOR = "rotation_vector"
    """[x, y, z, rx, ry, rz] — 6 values. Same as Nova's native format."""

    QUATERNION = "quaternion"
    """[x, y, z, qx, qy, qz, qw] — 7 values."""

    ROT6D = "rot6d"
    """[x, y, z, r1x, r1y, r1z, r2x, r2y, r2z] — 9 values. GR00T format."""


@dataclass
class FeatureGroup:
    """Maps a motion group to named features for the policy.

    The ``name`` defines the default prefix for all feature keys:

    - Joints: ``joint_key`` defaults to ``{name}_joint_position``
    - TCP:    ``tcp_key`` defaults to ``{name}_tcp``
    - IOs:    keys from the ``ios`` dict are used directly

    Array-based policies (GR00T) use these keys as dict keys.
    Flat-feature policies (LeRobot) expand joints to ``{joint_key}_{i}``.
    """

    motion_group: MotionGroup
    """The NOVA motion group this feature group controls."""

    name: str
    """Name prefix for feature keys, e.g. ``left``, ``right``, ``arm``."""

    ios: dict[str, str] | None = None
    """Named IO mapping: policy feature name → hardware IO key.

    Example: ``{"gripper_position": "digital_out[0]"}``

    The policy sees the dict key directly (e.g. ``gripper_position``).
    Actions targeting that key write to the mapped hardware IO.
    """

    io_threshold: float = 0.5
    """For boolean IOs: policy values >= threshold write True, below write False."""

    tcp_format: TcpFormat = TcpFormat.NONE
    """TCP pose representation to include in observations. Position is in meters."""

    joint_key: str = ""
    """Policy feature name for joints. Default: ``{name}_joint_position``.

    - Array policies (GR00T): used as dict key for the full joint array.
    - Flat policies (LeRobot): expanded to ``{joint_key}_{i}`` per joint.
    """

    tcp_key: str = ""
    """Policy feature name for TCP pose. Default: ``{name}_tcp``.

    E.g. ``"eef_9d"`` for GR00T OXE_DROID.
    """

    model_dof: int = 0
    """Number of joints the model expects. If the robot has fewer joints,
    zeros are appended. Extra joints in the action are dropped.
    0 = auto (use the robot's actual joint count)."""

    tcp: str = ""
    """TCP name for jogging and state streaming. Empty = robot's default."""

    @property
    def resolved_joint_key(self) -> str:
        """Resolved feature name for joints."""
        return self.joint_key or f"{self.name}_joint_position"

    @property
    def resolved_tcp_key(self) -> str:
        """Resolved feature name for TCP pose. Empty if tcp_format is NONE."""
        if not self.tcp_format:
            return ""
        return self.tcp_key or f"{self.name}_tcp"

    def resolved_io_key(self, io_name: str) -> str:
        """Resolved feature name for a named IO.

        IO names from the ``ios`` dict are used directly — they already
        represent the policy's vocabulary (e.g. ``gripper_position``).
        """
        return io_name

    def flat_joint_key(self, joint_index: int) -> str:
        """Return the flat (per-scalar) feature key for a 1-based joint index."""
        return f"{self.resolved_joint_key}_{joint_index}"

    def io_feature_key(self, io_name: str) -> str:
        """Return the flat feature key for a named IO (same as resolved_io_key)."""
        return self.resolved_io_key(io_name)

    @property
    def io_hardware_keys(self) -> list[str]:
        """All hardware IO keys to stream."""
        if self.ios is None:
            return []
        return list(self.ios.values())


@dataclass
class FeatureMap:
    """Static mapping between flat feature dicts and NOVA motion groups.

    Pure data — no lifecycle, no I/O, no streaming. Translates observations
    and actions between the policy's flat-feature vocabulary and NOVA's
    per-motion-group structure.
    """

    groups: list[FeatureGroup]

    def get_motion_groups(self) -> list[MotionGroup]:
        """Get all motion groups in order."""
        return [group.motion_group for group in self.groups]

    @property
    def tcp(self) -> str:
        """TCP name from the first group that has one set (or empty for default)."""
        for group in self.groups:
            if group.tcp:
                return group.tcp
        return ""

    def io_keys_by_controller(self) -> dict[str, list[str]]:
        """Return hardware IO keys grouped by controller ID.

        Used by the executor to set up IO streams.
        Returns: {controller_id: [io_key, ...]}
        """
        result: dict[str, set[str]] = {}
        for group in self.groups:
            controller_id = get_controller_id(group.motion_group)
            if controller_id not in result:
                result[controller_id] = set()
            result[controller_id].update(group.io_hardware_keys)
        return {k: sorted(v) for k, v in result.items() if v}

    def build_grouped_observation(
        self,
        states: dict[str, RobotState],
        io_values: dict[str, object] | None = None,
    ) -> list[GroupObservation]:
        """Extract per-group observation data from robot states.

        This is the shared extraction step used by both flat-feature policies
        (via ``build_observation()``) and array-based policies (GR00T).

        Returns one ``GroupObservation`` per group with joints, TCP, and IO values.
        """
        result: list[GroupObservation] = []
        for group in self.groups:
            state = states.get(group.motion_group.id)
            if state is None:
                continue

            joints = list(state.joints)
            tcp_values: list[float] | None = None
            if group.tcp_format and hasattr(state, "pose") and state.pose is not None:
                tcp_values = pose_to_tcp(state.pose, group.tcp_format)

            io_floats: dict[str, float] | None = None
            if group.ios is not None and io_values is not None:
                io_floats = {}
                for name, hw_key in group.ios.items():
                    raw_value = io_values.get(hw_key)
                    if raw_value is None:
                        io_floats[name] = 0.0
                    elif isinstance(raw_value, bool):
                        io_floats[name] = 1.0 if raw_value else 0.0
                    else:
                        io_floats[name] = float(raw_value)

            result.append(GroupObservation(
                group=group, joints=joints, tcp=tcp_values, ios=io_floats,
            ))
        return result

    def build_observation(
        self,
        states: dict[str, RobotState],
        io_values: dict[str, object] | None = None,
    ) -> dict[str, float]:
        """Convert per-group RobotState into a flat feature dict.

        Args:
            states: Motion group ID → current RobotState.
            io_values: Hardware IO key → current value (from IO stream cache).
                       If None, IOs are omitted from the observation.
        """
        obs: dict[str, float] = {}
        for gobs in self.build_grouped_observation(states, io_values):
            group = gobs.group
            for i, v in enumerate(gobs.joints, start=1):
                obs[group.flat_joint_key(i)] = v
            if gobs.tcp is not None:
                tcp_key = group.resolved_tcp_key
                for i, v in enumerate(gobs.tcp, start=1):
                    obs[f"{tcp_key}_{i}"] = v
            if gobs.ios is not None:
                for name, value in gobs.ios.items():
                    obs[group.io_feature_key(name)] = value
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

            # Joints — collect as many as are present in the action
            joint_values: list[float] = []
            joint_index = 1
            while True:
                key = group.flat_joint_key(joint_index)
                if key not in action:
                    break
                joint_values.append(float(action[key]))
                joint_index += 1

            if joint_values:
                joints[motion_group_id] = [joint_values]

            # IOs
            if group.ios is None:
                continue
            group_ios: dict[str, bool | int | float | str] = {}
            for name, hw_key in group.ios.items():
                feature_key = group.io_feature_key(name)
                if feature_key not in action:
                    continue
                value = float(action[feature_key])
                group_ios[hw_key] = value >= group.io_threshold

            if group_ios:
                ios[motion_group_id] = group_ios

        return joints, ios or None
