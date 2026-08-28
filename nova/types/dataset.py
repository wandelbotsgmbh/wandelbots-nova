from os import PathLike
from typing import Annotated, Literal

from pydantic import Field
from pydantic.dataclasses import dataclass

from nova import api


class Dataset(api.models.Dataset):
    """A Dataset with its persisted poses, frames and command routines.
    Convenience class that equals the according GetDatasetResponse from the API, but with the poses, command routines and frames as dictionaries instead of lists.
    """

    poses: dict[api.models.DatasetPoseId, api.models.DatasetPose] = Field(default_factory=dict)
    command_routines: dict[api.models.CommandRoutineId, api.models.CommandRoutine] = Field(
        default_factory=dict
    )
    frames: dict[api.models.FrameId, api.models.DatasetFrame] = Field(default_factory=dict)


@dataclass(frozen=True)
class LoadRemoteDatasetRequest:
    """
    Request to load a dataset that is stored on the NOVA instance.
    """

    dataset: api.models.DatasetId
    revision: int | None = None
    type: Literal["remote"] = "remote"


@dataclass(frozen=True)
class LoadLocalDatasetRequest:
    """
    Request to load a dataset from a local JSON file.

    `path` is kept exactly as written. A relative one is resolved only when the
    dataset is loaded, against the file of the ``@nova.program`` that declares it -
    so this request stays portable across machines when it is serialized and
    published to a program store.
    """

    path: PathLike
    type: Literal["local"] = "local"


LoadDatasetRequest = Annotated[
    LoadRemoteDatasetRequest | LoadLocalDatasetRequest, Field(discriminator="type")
]
