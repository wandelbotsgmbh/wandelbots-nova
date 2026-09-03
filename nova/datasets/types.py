from pathlib import Path
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

    path: Path
    type: Literal["local"] = "local"


LoadDatasetRequest = Annotated[
    LoadRemoteDatasetRequest | LoadLocalDatasetRequest, Field(discriminator="type")
]


def remote_dataset(
    dataset: api.models.DatasetId, revision: int | None = None
) -> LoadRemoteDatasetRequest:
    """Create a configuration for loading a dataset stored on the NOVA instance.

    Args:
        dataset: Identifier of the dataset to load.
        revision: Revision to load. When omitted, the latest revision is used.
    """
    return LoadRemoteDatasetRequest(dataset=dataset, revision=revision)


def local_dataset(path: str) -> LoadLocalDatasetRequest:
    """Create a configuration for loading a dataset from a local JSON file.

    `path` should be a plain string literal, e.g. ``ds.local_dataset("my_dataset.json")``
    - not an expression such as ``Path(__file__).parent / "..."``, since external
    tooling reads it directly from the program's source code.

    A relative `path` is stored as written and resolved when the dataset is loaded,
    against the file of the ``@nova.program`` that declares it.

    Raises:
        ValueError: If `path` is absolute.
    """
    if Path(path).is_absolute():
        raise ValueError(
            f"local_dataset() path must be relative, got absolute path '{path}'. Relative paths "
            "are resolved against the file of the @nova.program that declares them, so the "
            "dataset is found next to its program on any machine."
        )
    return LoadLocalDatasetRequest(path=Path(path))
