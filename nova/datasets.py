import asyncio
import logging
from pathlib import Path

from nova import api
from nova.core.nova import Nova
from nova.types.dataset import (
    Dataset,
    LoadDatasetRequest,
    LoadLocalDatasetRequest,
    LoadRemoteDatasetRequest,
)

logger = logging.getLogger(__name__)


def _dataset_from_api(api_dataset: api.models.GetDatasetResponse) -> Dataset:
    """Convert the api datasets response into the convinience class Dataset"""
    return Dataset(
        **api_dataset.model_dump(exclude={"poses", "command_routines", "frames"}),
        poses={pose.dataset_pose: pose for pose in api_dataset.poses} if api_dataset.poses else {},
        command_routines={
            routine.command_routine: routine for routine in api_dataset.command_routines
        }
        if api_dataset.command_routines
        else {},
        frames={frame.frame: frame for frame in api_dataset.frames} if api_dataset.frames else {},
    )


async def list_datasets(
    nova: Nova, dataset_id: api.models.DatasetId | None = None, latest_only: bool | None = None
) -> list[api.models.Dataset]:
    """List the datasets available in the cell on the NOVA instance.

    Every revision is returned as its own entry unless the result is narrowed down.

    Args:
        nova: A connected NOVA instance.
        dataset_id: Restrict the result to all revisions of this dataset.
        latest_only: When True, return only the latest revision of each dataset.
    """
    assert nova.is_connected(), "NOVA instance needs to be connected, in order to list datasets."

    try:
        return await nova.api.datasets_api.get_datasets(
            cell=nova.cell().id,
            dataset=str(dataset_id) if dataset_id is not None else None,
            latest_only=latest_only,
        )
    except Exception:
        logger.exception("Failed to list datasets")
        raise


async def create_dataset(nova: Nova, create_request: api.models.CreateDatasetRequest) -> Dataset:
    """Create a dataset together with its poses, frames and command routines.

    Args:
        nova: A connected NOVA instance.
        create_request: The dataset to create. Server-managed fields are assigned by
            the NOVA instance and must not be part of the request.
    """
    assert nova.is_connected(), "NOVA instance needs to be connected, in order to create a dataset."

    try:
        response = await nova.api.datasets_api.create_dataset(
            cell=nova.cell().id, create_dataset_request=create_request
        )
        return _dataset_from_api(response)
    except Exception:
        logger.exception(f"Failed to create dataset '{create_request.dataset}'")
        raise


async def delete_dataset(
    nova: Nova, dataset_id: api.models.DatasetId, revision: int | None = None
) -> None:
    """Delete a dataset from the NOVA instance."""
    assert nova.is_connected(), "NOVA instance needs to be connected, in order to delete a dataset."

    try:
        await nova.api.datasets_api.delete_dataset(
            cell=nova.cell().id, dataset=str(dataset_id), revision=revision
        )
    except Exception:
        logger.exception(f"Failed to delete dataset '{dataset_id}'")
        raise


async def localize_pose_from_world(
    nova: Nova,
    poses: list[api.models.Pose],
    frame: api.models.FrameId,
    dataset: api.models.DatasetId,
    revision: int | None = None,
) -> list[api.models.Pose]:
    """Localize poses that are expressed in the world frame into the given dataset frame."""

    if not len(poses):
        logger.warning("No dataset poses provided, returning empty list.")
        return []

    assert nova.is_connected(), (
        "NOVA instance needs to be connected, in order to resolve dataset poses."
    )

    return await nova.api.datasets_api.localize_dataset_frame_pose(
        cell=nova.cell().id, dataset=str(dataset), revision=revision, frame=str(frame), poses=poses
    )


async def resolve_to_world(
    nova: Nova,
    poses: list[api.models.Pose],
    frame: api.models.FrameId,
    dataset: api.models.DatasetId,
    revision: int | None = None,
) -> list[api.models.Pose]:
    """Resolve poses from the dataset to world coordinates."""
    if not len(poses):
        logger.warning("No dataset poses provided, returning empty list.")
        return []

    assert nova.is_connected(), (
        "NOVA instance needs to be connected, in order to resolve dataset poses."
    )

    return await nova.api.datasets_api.resolve_dataset_frame_pose(
        cell=nova.cell().id, dataset=str(dataset), revision=revision, frame=str(frame), poses=poses
    )


def _resolve_local_path(local_dataset: LoadLocalDatasetRequest, relative_to: Path | None) -> Path:
    """Resolve a local dataset path, anchoring a relative one on `relative_to`.

    Relative paths are deliberately not resolved against the working directory: the
    dataset belongs to the program that declares it, so that program's own directory
    is the only anchor that stays correct wherever the process is started from.
    """
    path = Path(local_dataset.path)
    if path.is_absolute():
        return path
    if relative_to is None:
        raise ValueError(
            f"Cannot resolve the relative dataset path '{path}': no directory to resolve it "
            "against. Relative paths are resolved against the file of the @nova.program that "
            "declares the dataset, so either load this through a program or use an absolute path."
        )
    return relative_to / path


async def load_dataset(
    nova: Nova, dataset_request: LoadDatasetRequest, *, relative_to: Path | None = None
) -> Dataset:
    """Load a dataset from the NOVA instance or from a local file.

    Args:
        nova: A connected NOVA instance.
        dataset_request: The dataset to load.
        relative_to: Directory a relative local path is resolved against. Programs pass
            the directory of the file that declares the dataset.
    """
    if dataset_request.type == "remote":
        # A remote dataset is addressed by its id, so it already carries the right one.
        return await fetch_remote_dataset(nova, dataset_request.dataset, dataset_request.revision)

    else:
        return await read_local_dataset(dataset_request, relative_to=relative_to)


async def fetch_remote_dataset(
    nova: Nova, dataset_id: api.models.DatasetId, revision: int | None = None
) -> Dataset:
    """Fetch a dataset from the NOVA server."""

    assert nova.is_connected(), "NOVA instance needs to be connected, in order to fetch a dataset."

    try:
        response = await nova.api.datasets_api.get_dataset(
            cell=nova.cell().id, dataset=str(dataset_id), revision=revision
        )
        return _dataset_from_api(response)
    except Exception:
        logger.exception(f"Failed to fetch dataset '{dataset_id}'")
        raise


async def read_local_dataset(
    local_dataset: LoadLocalDatasetRequest, *, relative_to: Path | None = None
) -> Dataset:
    """Read a local dataset from a JSON file.

    Args:
        local_dataset: The dataset file to read.
        relative_to: Directory a relative path is resolved against. Programs pass the
            directory of the file that declares the dataset.
    """
    path = _resolve_local_path(local_dataset, relative_to)

    try:
        data = await asyncio.to_thread(path.read_text)
        response = api.models.GetDatasetResponse.model_validate_json(data)
        return _dataset_from_api(response)
    except Exception:
        logger.exception(f"Failed to read local dataset from '{path}'")
        raise


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
    against the file of the ``@nova.program`` that declares it. So the dataset is
    found next to its program regardless of the working directory, and the request
    stays portable if it is serialized and run on another machine.
    """
    return LoadLocalDatasetRequest(path=Path(path))
