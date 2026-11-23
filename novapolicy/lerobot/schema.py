"""LeRobot feature metadata and flat action layout derived from PolicySchema."""

from __future__ import annotations

from collections import Counter
import logging
from typing import TYPE_CHECKING, Any, cast

from lerobot.configs.types import FeatureType
from lerobot.utils.constants import ACTION, OBS_IMAGES, OBS_STATE
from lerobot.utils.feature_utils import dataset_to_policy_features
import numpy as np

from novapolicy import training_range
from novapolicy.schema import tcp_value_names

if TYPE_CHECKING:
    from lerobot.configs.types import PolicyFeature

    from nova.types import RobotState
    from novapolicy.lerobot.config import LeRobotExecutionSettings
    from novapolicy.schema import PolicySchema

logger = logging.getLogger(__name__)

_IMAGE_NDIM = 3

ImageSizeDiff = tuple[str, tuple[int, ...], tuple[int, ...]]

# Fractional aspect-ratio difference treated as the same aspect. Covers rounding
# in odd frame sizes without letting 4:3 pass as 16:9.
_ASPECT_TOLERANCE = 0.01


class LeRobotSchema:
    """Translate PolicySchema observations and actions to LeRobot's flat schema."""

    def __init__(self) -> None:
        self._logged_missing_features = False

    @staticmethod
    def state_names(states: dict[str, RobotState], schema: PolicySchema) -> list[str]:
        """The flat observation.state, in a fixed order: joint positions, joint velocities, TCP
        poses, object poses, IOs, then the keys of ``Observation.computed`` (one value each).
        The checkpoint matches it by position."""
        names: list[str] = []
        for mapping in [*schema.joint_mappings, *schema.joint_velocity_mappings]:
            dof = sum(
                len(state.joints)
                for motion_group in mapping.sources
                if (state := states.get(motion_group.id)) is not None
            )
            names.extend(f"{mapping.key}_{index}" for index in range(1, dof + 1))
        for mapping in schema.tcp_mappings:
            state = states.get(mapping.source.id)
            if state is not None and state.pose is not None:
                names.extend(tcp_value_names(mapping))
        for pose in schema.pose_mappings:
            names.extend(tcp_value_names(pose))
        names.extend(mapping.key for mapping in schema.obs_io_mappings)
        names.extend(key for computed in schema.computed_observations for key in computed.keys)
        return names

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
        for targets, what in (
            (joint_group_ids, "multiple joint action targets for motion groups"),
            (tcp_group_ids, "multiple TCP action targets for motion groups"),
            (io_targets, "multiple actions for the same IO target"),
        ):
            duplicates = sorted(target for target, count in Counter(targets).items() if count > 1)
            if duplicates:
                raise ValueError(f"LeRobotPolicyClient found {what}: {duplicates}")

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
        action_width: int,
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
            if expected_width != action_width:
                problems.append(
                    f"action width is {action_width}, checkpoint expects {expected_width}"
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
    """Hold the first live observation, after the schema's operators, against the
    checkpoint's ``observation.state`` statistics (see :mod:`novapolicy.training_range`).
    Silent without statistics or when the state width disagrees with them.

    Raises:
        ValueError: A value is implausibly far outside the training range.
    """
    stats = settings.stats.get(OBS_STATE)
    if stats is None:
        return
    bounds = [stats.bounds(i) for i in range(len(stats.minimum or stats.mean or ()))]
    training_range.check_vectors(
        {OBS_STATE: [observation.get(name) for name in state_names]},
        {OBS_STATE: bounds},
        labels={OBS_STATE: state_names},
    )
