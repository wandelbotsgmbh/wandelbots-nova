"""LeRobot feature metadata and flat action layout derived from PolicySchema."""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
import logging
from typing import TYPE_CHECKING, Any, cast

from lerobot.utils.constants import OBS_IMAGES, OBS_STATE
import numpy as np

from novapolicy.types import ActionChunk

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from nova.types import RobotState
    from novapolicy.schema import PolicySchema

logger = logging.getLogger(__name__)

_IMAGE_NDIM = 3
_TCP_SUFFIXES = ("x", "y", "z", "rx", "ry", "rz")
_TCP_ACTION_DIM = len(_TCP_SUFFIXES)

JointActionSlice = tuple[str, slice]
TcpActionSlice = tuple[str, slice]
IOActionSlice = tuple[str, str, Any, slice]


@dataclass(slots=True, frozen=True)
class FlatActionLayout:
    """Schema-derived slices in a flat policy action vector."""

    joints: list[JointActionSlice]
    tcp: list[TcpActionSlice]
    ios: list[IOActionSlice]


class LeRobotSchema:
    """Translate PolicySchema observations and actions to LeRobot's flat schema."""

    def __init__(self, *, dt_ms: float) -> None:
        self._dt_ms = dt_ms
        self._logged_action_chunk_shape = False

    @staticmethod
    async def build_observation(
        states: dict[str, RobotState],
        schema: PolicySchema,
        images: dict[str, Any] | None,
        io_values: dict[str, object] | None,
    ) -> dict[str, Any]:
        observation = await schema.build_observation(states, io_values)
        if images:
            observation.update(images)
        return observation

    @staticmethod
    def state_names(states: dict[str, RobotState], schema: PolicySchema) -> list[str]:
        names: list[str] = []
        for mapping in schema.joint_mappings:
            dof = sum(
                len(state.joints)
                for motion_group in mapping.sources
                if (state := states.get(motion_group.id)) is not None
            )
            names.extend(f"{mapping.key}_{index}" for index in range(1, dof + 1))
        for mapping in schema.tcp_mappings:
            state = states.get(mapping.source.id)
            if state is not None and state.pose is not None:
                names.extend(f"{mapping.key}_{suffix}" for suffix in _TCP_SUFFIXES)
        names.extend(mapping.key for mapping in schema.obs_io_mappings)
        return names

    @staticmethod
    def action_layout(
        states: dict[str, RobotState],
        schema: PolicySchema,
    ) -> FlatActionLayout:
        joint_slices: list[JointActionSlice] = []
        offset = 0
        for _key, motion_groups in schema.joint_action_keys:
            for motion_group in motion_groups:
                state = states.get(motion_group.id)
                if state is None:
                    continue
                dof = len(state.joints)
                joint_slices.append((motion_group.id, slice(offset, offset + dof)))
                offset += dof

        tcp_slices: list[TcpActionSlice] = []
        for _key, motion_group in schema.tcp_action_keys:
            tcp_slices.append((motion_group.id, slice(offset, offset + _TCP_ACTION_DIM)))
            offset += _TCP_ACTION_DIM

        io_slices: list[IOActionSlice] = []
        for _key, motion_group, io, mapping in schema.io_action_keys:
            io_slices.append((motion_group.id, io, mapping, slice(offset, offset + 1)))
            offset += 1
        return FlatActionLayout(joints=joint_slices, tcp=tcp_slices, ios=io_slices)

    @staticmethod
    def validate_schema(schema: PolicySchema) -> None:
        joint_group_ids = [
            motion_group.id
            for _key, motion_groups in schema.joint_action_keys
            for motion_group in motion_groups
        ]
        tcp_group_ids = [motion_group.id for _key, motion_group in schema.tcp_action_keys]
        io_targets = [
            (motion_group.id, io) for _key, motion_group, io, _mapping in schema.io_action_keys
        ]

        duplicate_joint_groups = sorted(
            group_id for group_id, count in Counter(joint_group_ids).items() if count > 1
        )
        if duplicate_joint_groups:
            raise ValueError(
                "LeRobotPolicyClient found multiple joint action targets for motion groups: "
                f"{duplicate_joint_groups}"
            )

        duplicate_tcp_groups = sorted(
            group_id for group_id, count in Counter(tcp_group_ids).items() if count > 1
        )
        if duplicate_tcp_groups:
            raise ValueError(
                "LeRobotPolicyClient found multiple TCP action targets for motion groups: "
                f"{duplicate_tcp_groups}"
            )

        duplicate_io_targets = sorted(
            target for target, count in Counter(io_targets).items() if count > 1
        )
        if duplicate_io_targets:
            raise ValueError(
                "LeRobotPolicyClient found multiple actions for the same IO target: "
                f"{duplicate_io_targets}"
            )

        joint_groups = set(joint_group_ids)
        tcp_groups = set(tcp_group_ids)
        conflicting_groups = sorted(joint_groups & tcp_groups)
        if conflicting_groups:
            raise ValueError(
                "LeRobotPolicyClient cannot control a motion group with both joint and TCP "
                f"actions: {conflicting_groups}"
            )
        if not joint_groups and not tcp_groups:
            raise ValueError("LeRobotPolicyClient requires at least one joint or TCP action target")

    def features(
        self,
        schema: PolicySchema,
        state_names: list[str],
        images: dict[str, Any] | None,
    ) -> dict[str, dict[str, Any]]:
        features: dict[str, dict[str, Any]] = {
            OBS_STATE: {
                "dtype": "float32",
                "shape": (len(state_names),),
                "names": state_names,
            }
        }
        for key in sorted(schema.image_sources):
            features[f"{OBS_IMAGES}.{key}"] = {
                "dtype": "image",
                "shape": self._image_shape(key, images),
                "names": ["height", "width", "channels"],
            }
        return features

    def decode_timed_actions(
        self,
        actions: list[Any],
        layout: FlatActionLayout,
    ) -> ActionChunk:
        return self.decode_arrays(
            [self.action_to_array(timed_action.get_action()) for timed_action in actions],
            layout,
        )

    def decode_arrays(
        self,
        action_arrays: list[NDArray[np.float32]],
        layout: FlatActionLayout,
        *,
        action_timestep: int = -1,
        io_action_array: NDArray[np.float32] | None = None,
    ) -> ActionChunk:
        if not action_arrays:
            raise ValueError("LeRobot returned no action steps")

        joints: dict[str, list[list[float]]] = {
            motion_group_id: [] for motion_group_id, _action_slice in layout.joints
        }
        tcp: dict[str, list[list[float]]] = {
            motion_group_id: [] for motion_group_id, _action_slice in layout.tcp
        }
        for action in action_arrays:
            for motion_group_id, action_slice in layout.joints:
                joints[motion_group_id].append([float(value) for value in action[action_slice]])
            for motion_group_id, action_slice in layout.tcp:
                values = action[action_slice]
                if values.size != _TCP_ACTION_DIM:
                    msg = (
                        f"LeRobot TCP action for {motion_group_id!r} expected "
                        f"{_TCP_ACTION_DIM} values, got {values.size}"
                    )
                    raise ValueError(msg)
                tcp[motion_group_id].append([float(value) for value in values])

        if not self._logged_action_chunk_shape:
            logger.info(
                "First LeRobot action chunk: %d steps, action_dim=%d",
                len(action_arrays),
                int(action_arrays[0].size),
            )
            self._logged_action_chunk_shape = True

        ios: dict[str, dict[str, bool | int | float | str]] = {}
        if layout.ios:
            io_source = action_arrays[0] if io_action_array is None else io_action_array
            for motion_group_id, io, mapping, action_slice in layout.ios:
                values = io_source[action_slice]
                if values.size != 1:
                    msg = f"LeRobot IO action {io!r} expected one value, got {values.size}"
                    raise ValueError(msg)
                ios.setdefault(motion_group_id, {})[io] = mapping.to_hardware(float(values[0]))

        return ActionChunk(
            joints=joints,
            tcp=tcp,
            ios=ios or None,
            dt_ms=self._dt_ms,
            action_timestep=action_timestep,
        )

    @staticmethod
    def replace_motion_values(
        action: NDArray[np.float32],
        chunk: ActionChunk,
        layout: FlatActionLayout,
        *,
        step: int,
    ) -> NDArray[np.float32]:
        transformed = action.copy()
        for group_id, action_slice in layout.joints:
            transformed[action_slice] = chunk.joints[group_id][step]
        for group_id, action_slice in layout.tcp:
            transformed[action_slice] = chunk.tcp[group_id][step]
        return transformed

    @staticmethod
    def action_to_array(action: object) -> NDArray[np.float32]:
        if hasattr(action, "detach"):
            action = cast("Any", action).detach().cpu().numpy()
        return np.asarray(action, dtype=np.float32).reshape(-1)

    @staticmethod
    def _image_shape(key: str, images: dict[str, Any] | None) -> tuple[int, int, int]:
        image = images.get(key) if images is not None else None
        if isinstance(image, np.ndarray) and image.ndim == _IMAGE_NDIM:
            return cast("tuple[int, int, int]", tuple(int(value) for value in image.shape))
        msg = (
            f"LeRobot image observation {key!r} is missing or is not an HxWxC numpy array. "
            "The client needs the first camera frame to declare LeRobot feature metadata; "
            "configure camera resolution with WebRTCCameras(..., resize=(width, height))."
        )
        raise ValueError(msg)
