from os import PathLike
from typing import Annotated, Literal

from pydantic import Field
from pydantic.dataclasses import dataclass

from nova import api


class Dataset(api.models.GetDatasetResponse):
    """A Dataset with its persisted poses, coordinate systems and command routines."""

    poses: dict[api.models.DatasetPoseId, api.models.DatasetPose] | None = None
    command_routines: dict[api.models.CommandRoutineId, api.models.CommandRoutine] | None = None
    coordinate_systems: dict[api.models.CoordinateSystemId, api.models.CoordinateSystem2] | None = (
        None
    )

    @classmethod
    def from_api_model(cls, api_dataset: api.models.GetDatasetResponse) -> "Dataset":
        return cls(
            **api_dataset.model_dump(exclude={"poses", "command_routines", "coordinate_systems"}),
            poses={pose.dataset_pose: pose for pose in api_dataset.poses}
            if api_dataset.poses
            else None,
            command_routines={
                routine.command_routine: routine for routine in api_dataset.command_routines
            }
            if api_dataset.command_routines
            else None,
            coordinate_systems={cs.coordinate_system: cs for cs in api_dataset.coordinate_systems}
            if api_dataset.coordinate_systems
            else None,
        )


@dataclass(frozen=True)
class LoadRemoteDatasetRequest:
    dataset: api.models.DatasetId
    revision: int | None = None
    override_dataset_id: str | None = None
    type: Literal["remote"] = "remote"


@dataclass(frozen=True)
class LoadLocalDatasetRequest:
    path: PathLike
    override_dataset_id: str | None = None
    type: Literal["local"] = "local"


LoadDatasetRequest = Annotated[
    LoadRemoteDatasetRequest | LoadLocalDatasetRequest, Field(discriminator="type")
]
