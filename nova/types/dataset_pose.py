from typing import Annotated

import pydantic
from pydantic import AwareDatetime

from nova import api

# TODO: Remove these compatibility models once `DatasetPose` is available in the minimum
# supported version of `wandelbots_api_client` which will be version 26.6.0. Until then, keep its fields aligned with
# the API schema so SDK users can adopt dataset poses before the generated model is released.

_ID_PATTERN = r"^[A-Za-z][A-Za-z0-9-]*$"

DatasetId = Annotated[str, pydantic.StringConstraints(pattern=_ID_PATTERN)]
"""
Unique identifier of the dataset.
Must start with a letter and may only contain letters, digits, and hyphens.
"""

DatasetPoseId = Annotated[str, pydantic.StringConstraints(pattern=_ID_PATTERN)]
"""
Unique identifier of the pose within the dataset.
Must start with a letter and may only contain letters, digits, and hyphens.
"""

CoordinateSystemId = str
"""
Unique identifier of a coordinate system.
"""


class ConfiguredPose(pydantic.BaseModel):
    """
    A pose together with an optional kinematic configuration that resolves it to a unique
    robot posture. Optionally, the pose can be expressed relative to a coordinate system.
    """

    pose: api.models.Pose
    kinematic_configuration: api.models.KinematicConfiguration | None = None
    coordinate_system: CoordinateSystemId | None = None
    """
    Optional identifier of the coordinate system the pose is expressed in. If this
    is null or omitted, the pose is referenced in `world`.
    """


class DatasetPose(ConfiguredPose):
    """
    A dataset pose is a taught pose that belongs to a dataset. It extends the
    shared `ConfiguredPose` with dataset-specific fields such as an identifier, human-readable
    name, timestamps, and metadata.
    """

    dataset_pose: DatasetPoseId
    """
    Unique identifier of the pose within the dataset.
    """
    dataset: DatasetId
    """
    Identifier of the dataset that contains this pose.
    """
    name: str | None = None
    """
    Human-readable name for the pose that is shown in the user interface.
    """
    description: str | None = None
    """
    Free-form text that describes the purpose of the pose.
    """
    created_at: AwareDatetime | None = None
    """
    Timestamp when the pose was created, in RFC3339 format.
    """
    updated_at: AwareDatetime | None = None
    """
    Timestamp when the pose was last updated, in RFC3339 format.
    """
    metadata: dict[str, str] | None = None
    """
    Additional key-value pairs that can be attached to the pose.
    """
