"""Load, create and query NOVA datasets - poses, frames and command routines
grouped under a named, revisioned resource.
"""

from nova.datasets.datasets import (
    fetch,
    read,
    resolve_command_routine,
    transform_to_frame,
    transform_to_world,
)
from nova.datasets.exceptions import DatasetError, DatasetNotFoundError, FrameResolutionError
from nova.datasets.types import (
    Dataset,
    DatasetFrame,
    DatasetPose,
    LoadDatasetRequest,
    LoadLocalDatasetRequest,
    LoadRemoteDatasetRequest,
    local_dataset,
    remote_dataset,
)

__all__ = [
    "Dataset",
    "DatasetError",
    "DatasetFrame",
    "DatasetNotFoundError",
    "DatasetPose",
    "FrameResolutionError",
    "LoadDatasetRequest",
    "LoadLocalDatasetRequest",
    "LoadRemoteDatasetRequest",
    "fetch",
    "read",
    "resolve_command_routine",
    "remote_dataset",
    "local_dataset",
    # Deprecated: frames resolve locally via DatasetPose.as_world() / DatasetFrame.as_world().
    "transform_to_frame",
    "transform_to_world",
]
