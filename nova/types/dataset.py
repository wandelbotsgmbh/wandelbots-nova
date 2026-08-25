from os import PathLike
from typing import Annotated, Literal

from pydantic import Field
from pydantic.dataclasses import dataclass

from nova import api


class Dataset(api.models.Dataset):
    """A Dataset with its persisted poses, coordinate systems and command routines.
    Convenience class that equals the according GetDatasetResponse from the API, but with the poses, command routines and coordinate systems as dictionaries instead of lists.
    """

    poses: dict[api.models.DatasetPoseId, api.models.DatasetPose] = Field(default_factory=dict)
    command_routines: dict[api.models.CommandRoutineId, api.models.CommandRoutine] = Field(
        default_factory=dict
    )
    coordinate_systems: dict[api.models.CoordinateSystemId, api.models.DatasetCoordinateSystem] = (
        Field(default_factory=dict)
    )


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
    """

    path: PathLike
    type: Literal["local"] = "local"


LoadDatasetRequest = Annotated[
    LoadRemoteDatasetRequest | LoadLocalDatasetRequest, Field(discriminator="type")
]
