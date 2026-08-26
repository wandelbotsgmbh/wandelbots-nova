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
        **api_dataset.model_dump(exclude={"poses", "command_routines", "coordinate_systems"}),
        poses={pose.dataset_pose: pose for pose in api_dataset.poses} if api_dataset.poses else {},
        command_routines={
            routine.command_routine: routine for routine in api_dataset.command_routines
        }
        if api_dataset.command_routines
        else {},
        coordinate_systems={cs.coordinate_system: cs for cs in api_dataset.coordinate_systems}
        if api_dataset.coordinate_systems
        else {},
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
    """Create a dataset together with its poses, coordinate systems and command routines.

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
    coordinate_system: api.models.CoordinateSystemId,
    dataset: api.models.DatasetId,
    revision: int | None = None,
) -> list[api.models.Pose]:
    """Localize poses that are expressed in the world coordinate system into the given dataset coordinate system."""

    if not len(poses):
        logger.warning("No dataset poses provided, returning empty list.")
        return []

    assert nova.is_connected(), (
        "NOVA instance needs to be connected, in order to resolve dataset poses."
    )

    return await nova.api.datasets_api.localize_dataset_coordinate_system_pose(
        cell=nova.cell().id,
        dataset=str(dataset),
        revision=revision,
        coordinate_system=str(coordinate_system),
        poses=poses,
    )


async def resolve_to_world(
    nova: Nova,
    poses: list[api.models.Pose],
    coordinate_system: api.models.CoordinateSystemId,
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

    return await nova.api.datasets_api.resolve_dataset_coordinate_system_pose(
        cell=nova.cell().id,
        dataset=str(dataset),
        revision=revision,
        coordinate_system=str(coordinate_system),
        poses=poses,
    )


async def load_dataset(nova: Nova, dataset_request: LoadDatasetRequest) -> Dataset:
    """Load a dataset from the NOVA instance or from a local file."""
    if dataset_request.type == "remote":
        # A remote dataset is addressed by its id, so it already carries the right one.
        return await fetch_remote_dataset(nova, dataset_request.dataset, dataset_request.revision)

    else:
        return await read_local_dataset(dataset_request)


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


async def read_local_dataset(local_dataset: LoadLocalDatasetRequest) -> Dataset:
    """Read a local dataset from a JSON file"""

    try:
        data = await asyncio.to_thread(Path(local_dataset.path).read_text)
        response = api.models.GetDatasetResponse.model_validate_json(data)
        return _dataset_from_api(response)
    except Exception:
        logger.exception(f"Failed to read local dataset from '{local_dataset.path}'")
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
    """Create a configuration for loading a dataset from a local JSON file."""
    return LoadLocalDatasetRequest(path=Path(path))
