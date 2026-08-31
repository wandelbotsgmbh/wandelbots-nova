"""LeRobot feature metadata and flat action layout derived from PolicySchema."""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
import logging
from typing import TYPE_CHECKING, Any, cast

from lerobot.configs.types import FeatureType
from lerobot.utils.constants import ACTION, OBS_IMAGES, OBS_STATE
from lerobot.utils.feature_utils import dataset_to_policy_features
import numpy as np

from novapolicy.types import ActionChunk

if TYPE_CHECKING:
    from lerobot.configs.types import PolicyFeature
    from numpy.typing import NDArray

    from nova.types import RobotState
    from novapolicy.lerobot.config import FeatureStats, LeRobotExecutionSettings
    from novapolicy.schema import PolicySchema

logger = logging.getLogger(__name__)

_IMAGE_NDIM = 3
# A value this many training spans outside the range is worth mentioning: it is
# a pose the demonstrations never visited, which is normal before homing.
_WARN_MARGIN = 1.0

# ...whereas a value this many times larger than anything in training is not a
# pose at all. A units error is an error of *scale* — radians read as degrees is
# ~57x — so it is caught by magnitude, not by span. Scaling the failure
# threshold by span instead was measured wrong against a live UR10e: a joint
# that barely moved during training (0.022 rad) got a +/-0.09 rad window, and
# the arm's own park pose was reported as a unit mismatch.
_FAIL_SCALE = 5.0
_TCP_SUFFIXES = ("x", "y", "z", "rx", "ry", "rz")
_TCP_ACTION_DIM = len(_TCP_SUFFIXES)

JointActionSlice = tuple[str, slice]
TcpActionSlice = tuple[str, slice]
IOActionSlice = tuple[str, str, Any, slice]
ImageSizeDiff = tuple[str, tuple[int, ...], tuple[int, ...]]

# Fractional aspect-ratio difference treated as the same aspect. Covers rounding
# in odd frame sizes without letting 4:3 pass as 16:9.
_ASPECT_TOLERANCE = 0.01


@dataclass(slots=True, frozen=True)
class FlatActionLayout:
    """Schema-derived slices in a flat policy action vector."""

    joints: list[JointActionSlice]
    tcp: list[TcpActionSlice]
    ios: list[IOActionSlice]

    @property
    def width(self) -> int:
        """Number of values in the flat action vector this layout describes."""
        stops = [action_slice.stop for _group, action_slice in self.joints]
        stops.extend(action_slice.stop for _group, action_slice in self.tcp)
        stops.extend(action_slice.stop for _group, _io, _mapping, action_slice in self.ios)
        return max(stops, default=0)


class LeRobotSchema:
    """Translate PolicySchema observations and actions to LeRobot's flat schema."""

    def __init__(self, *, dt_ms: float) -> None:
        self._dt_ms = dt_ms
        self._logged_action_chunk_shape = False
        self._logged_missing_features = False

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

    def assert_matches(
        self,
        settings: LeRobotExecutionSettings,
        schema: PolicySchema,
        state_names: list[str],
        images: dict[str, Any] | None,
        layout: FlatActionLayout,
    ) -> None:
        """Check the schema against what the checkpoint expects.

        Compares the feature set this client is about to declare against the
        checkpoint's own ``input_features`` / ``output_features``: the key set,
        the shape behind each key, and the width of the flat action vector.

        Both sides are normalized through LeRobot's ``dataset_to_policy_features``
        so images are compared channel-first, the way the checkpoint stores them.

        Only ``STATE`` and ``VISUAL`` inputs are compared. A checkpoint may also
        declare ``LANGUAGE`` or ``ENV`` features that a ``PolicySchema`` never
        emits as observations; demanding those would fail every task-conditioned
        checkpoint spuriously.

        A **camera frame size** that disagrees only warns, and says something
        different for a resolution difference than for an aspect-ratio one. The
        frame size belongs to the camera stream, not to the schema, and hardware
        cannot always deliver an arbitrary resolution — so it stays the user's
        call. Everything else — a missing or unexpected observation key, a wrong
        state width, a wrong action width — raises.

        A checkpoint that declares no comparable features (they are inferred from
        the dataset at train time) is warned about once and skipped, never
        silently passed.

        Raises:
            ValueError: The schema cannot satisfy the checkpoint's contract.
        """
        problems: list[str] = []
        expected_inputs = settings.comparable_input_features
        if expected_inputs:
            actual = dataset_to_policy_features(
                cast("dict[str, Any]", self.features(schema, state_names, images))
            )
            structural, image_sizes = _compare_features(expected_inputs, actual)
            problems.extend(structural)
            for key, actual_shape, expected_shape in image_sizes:
                _log_image_size_difference(key, actual_shape, expected_shape)
        elif not self._logged_missing_features:
            logger.warning(
                "LeRobot checkpoint declares no input features; the schema cannot be "
                "validated against it. Observation names, ordering and image sizes must "
                "match the training dataset."
            )
            self._logged_missing_features = True

        expected_action = settings.output_features.get(ACTION)
        if expected_action is not None:
            expected_width = expected_action.shape[0] if expected_action.shape else 0
            if expected_width != layout.width:
                problems.append(
                    f"action width is {layout.width}, checkpoint expects {expected_width}"
                )

        if problems:
            detail = "\n  - ".join(problems)
            msg = (
                "PolicySchema does not match the LeRobot checkpoint:\n  - "
                f"{detail}\n"
                "Fix the schema, or point the client at the checkpoint the policy "
                "server actually loads."
            )
            raise ValueError(msg)

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


def _compare_features(
    expected: dict[str, PolicyFeature],
    actual: dict[str, PolicyFeature],
) -> tuple[list[str], list[ImageSizeDiff]]:
    """Describe every way ``actual`` fails to satisfy ``expected``.

    Returns ``(structural, image_sizes)``. Structural problems are the ones the
    schema alone can fix — a missing key, an unexpected key, a wrong state width
    — and are returned as fatal message lines. Image-size differences are
    returned as ``(key, actual_shape, expected_shape)`` instead, because they are
    fixed at the camera stream and the caller decides how loudly to say so.
    """
    structural: list[str] = [
        f"{key!r} is missing from the schema (checkpoint shape {expected[key].shape})"
        for key in sorted(set(expected) - set(actual))
    ]
    structural.extend(
        f"{key!r} is not declared by the checkpoint (schema shape {actual[key].shape})"
        for key in sorted(set(actual) - set(expected))
    )

    image_sizes: list[ImageSizeDiff] = []
    for key in sorted(set(expected) & set(actual)):
        if expected[key].shape == actual[key].shape:
            continue
        if expected[key].type is FeatureType.VISUAL:
            image_sizes.append((key, actual[key].shape, expected[key].shape))
        else:
            structural.append(
                f"{key!r} has shape {actual[key].shape}, checkpoint expects {expected[key].shape}"
            )
    return structural, image_sizes


def _frame_size(shape: tuple[int, ...]) -> tuple[int, int] | None:
    """``(width, height)`` from a channel-first ``(C, H, W)`` feature shape."""
    if len(shape) != _IMAGE_NDIM:
        return None
    return int(shape[2]), int(shape[1])


def _describe(size: tuple[int, int]) -> str:
    width, height = size
    return f"{width}x{height} (aspect {width / height:.2f})" if height else f"{width}x{height}"


def _log_image_size_difference(
    key: str,
    actual_shape: tuple[int, ...],
    expected_shape: tuple[int, ...],
) -> None:
    """Warn about a camera frame that differs from the checkpoint's.

    Separates the two cases deliberately. A different *resolution* at the same
    aspect ratio rescales cleanly, so it costs bandwidth and detail — worth
    saying, not alarming. A different *aspect ratio* cannot be corrected by
    rescaling: the image is stretched, and a policy without internal padding
    (ACT among them) sees geometry that never appeared in training.
    """
    actual_size = _frame_size(actual_shape)
    expected_size = _frame_size(expected_shape)
    if actual_size is None or expected_size is None:
        logger.warning(
            "Camera %r delivers shape %s but the checkpoint expects %s.",
            key,
            actual_shape,
            expected_shape,
        )
        return

    actual_aspect = actual_size[0] / actual_size[1]
    expected_aspect = expected_size[0] / expected_size[1]
    if abs(actual_aspect - expected_aspect) > _ASPECT_TOLERANCE * expected_aspect:
        logger.warning(
            "Camera %r has a different aspect ratio than the checkpoint: frames are %s, "
            "the checkpoint expects %s. Rescaling cannot correct this — the image is "
            "stretched, and policies without internal padding (ACT among them) see "
            "geometry that never appeared in training. Configure the camera stream at a "
            "matching aspect ratio.",
            key,
            _describe(actual_size),
            _describe(expected_size),
        )
        return

    logger.warning(
        "Camera %r delivers %s but the checkpoint expects %s. The aspect ratio matches, "
        "so frames rescale cleanly; configuring the stream at the checkpoint's resolution "
        "avoids sending pixels that are only downscaled again.",
        key,
        _describe(actual_size),
        _describe(expected_size),
    )


def check_observation_range(
    settings: LeRobotExecutionSettings,
    observation: dict[str, Any],
    state_names: list[str],
) -> None:
    """Hold the first live observation against the checkpoint's own statistics.

    Declared units are an assertion by whoever wrote the schema; the checkpoint
    can settle it. A joint reported in degrees against a checkpoint trained on
    radians is a ~57x error, and lands so far outside the training distribution
    that nothing else it could be — wrong units, wrong joint order, wrong robot,
    wrong TCP — matters to the diagnosis.

    Values are compared *after* the schema's operators, because that is what the
    policy will actually see. Two thresholds, and deliberately different in kind:
    sitting outside the demonstrated *range* only warns, because a robot parked
    somewhere the demonstrations never visited is normal; being an order of
    *magnitude* larger than anything in training raises, because that is not a
    pose, it is a unit.

    Silent when the checkpoint ships no statistics, or when the state width
    disagrees with them — the feature contract check owns that failure.

    Raises:
        ValueError: A value is implausibly far outside the training range.
    """
    stats = settings.stats.get(OBS_STATE)
    if stats is None:
        return

    width = _stats_width(stats)
    if width is None or width != len(state_names):
        logger.debug(
            "Skipping the observation range check: %d state names against %s statistics.",
            len(state_names),
            width,
        )
        return

    problems, warnings = _classify_observations(stats, observation, state_names)

    for note in warnings:
        logger.warning(
            "Observation outside the training distribution: %s. Normal at a start pose the "
            "demonstrations never visited; suspicious otherwise.",
            note,
        )

    if problems:
        detail = "\n  - ".join(problems)
        msg = (
            "Observations are implausibly far outside what this checkpoint was trained on:"
            f"\n  - {detail}\n"
            "This is what a unit mismatch looks like — check whether the dataset was "
            "recorded in different units than NOVA reports, and declare the conversion "
            "with ops= on the observation."
        )
        raise ValueError(msg)


def _stats_width(stats: FeatureStats) -> int | None:
    for values in (stats.minimum, stats.mean):
        if values is not None:
            return len(values)
    return None


def _classify_observations(
    stats: FeatureStats,
    observation: dict[str, Any],
    state_names: list[str],
) -> tuple[list[str], list[str]]:
    """Split state values into implausible ones and merely unusual ones."""
    problems: list[str] = []
    warnings: list[str] = []
    for index, name in enumerate(state_names):
        value = observation.get(name)
        if not isinstance(value, (int, float)) or isinstance(value, bool):
            continue
        bounds = stats.bounds(index)
        if bounds is None:
            continue
        low, high = bounds
        span = high - low
        magnitude = max(abs(low), abs(high))
        if span <= 0 and magnitude <= 0:
            continue
        note = f"{name!r} is {float(value):.4g}, training range [{low:.4g}, {high:.4g}]"
        if magnitude > 0 and abs(value) > _FAIL_SCALE * magnitude:
            problems.append(note)
        elif span > 0 and not (low - _WARN_MARGIN * span <= value <= high + _WARN_MARGIN * span):
            warnings.append(note)
    return problems, warnings
