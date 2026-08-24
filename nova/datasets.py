import logging
from pathlib import Path

from nova import api
from nova.config import CELL_NAME
from nova.core.nova import Nova
from nova.types.dataset import LoadDatasetRequest, LoadLocalDatasetRequest, LoadRemoteDatasetRequest

logger = logging.getLogger(__name__)


def remote_dataset(
    dataset: api.models.DatasetId,
    revision: int | None = None,
    override_dataset_id: str | None = None,
) -> LoadDatasetRequest:
    """Load a dataset from the remote server."""
    return LoadRemoteDatasetRequest(
        dataset=dataset, revision=revision, override_dataset_id=override_dataset_id
    )


def local_dataset(path: str, override_dataset_id: str | None = None) -> LoadDatasetRequest:
    """Load a dataset from a local file."""
    return LoadLocalDatasetRequest(path=Path(path), override_dataset_id=override_dataset_id)


async def localize_pose_from_world(
    nova: Nova,
    poses: list[api.models.Pose],
    coordinate_system: api.models.CoordinateSystemId,
    dataset: api.models.DatasetId,
    revision: int | None = None,
) -> list[api.models.Pose]:
    """Localize poses that are expressed in the world coordinate system into the given dataset coordinate system."""

    assert nova.is_connected(), (
        "NOVA instance needs to be connected, in order to resolve dataset poses."
    )

    if not len(poses):
        logger.warning("No dataset poses provided, returning empty list.")
        return []

    return await nova.api.datasets_api.localize_dataset_coordinate_system_pose(
        cell=CELL_NAME,
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
        cell=CELL_NAME,
        dataset=str(dataset),
        revision=revision,
        coordinate_system=str(coordinate_system),
        poses=poses,
    )


async def load_dataset(
    nova: Nova, dataset_request: LoadDatasetRequest
) -> api.models.GetDatasetResponse:
    """Load a dataset from the remote server or from a local file."""
    if dataset_request.type == "remote":
        return await fetch_remote_dataset(nova, dataset_request.dataset, dataset_request.revision)
    else:
        return await read_local_dataset(dataset_request)


async def fetch_remote_dataset(
    nova: Nova, dataset_id: api.models.DatasetId, revision: int | None = None
) -> api.models.GetDatasetResponse:
    """Load the complete dataset from the API."""

    assert nova.is_connected(), "NOVA instance needs to be connected, in order to fetch a dataset."

    try:
        response: api.models.GetDatasetResponse = await nova.api.datasets_api.get_dataset(
            cell=CELL_NAME, dataset=str(dataset_id), revision=revision
        )
    except Exception as e:
        logger.error(f"Failed to fetch dataset '{dataset_id}': {e}")
        raise

    return api.models.GetDatasetResponse.model_validate(response.model_dump())


async def read_local_dataset(
    local_dataset: LoadLocalDatasetRequest,
) -> api.models.GetDatasetResponse:
    """Read a local dataset from a file."""

    try:
        with open(local_dataset.path, "r") as f:
            data = f.read()
        dataset = api.models.GetDatasetResponse.model_validate_json(data)
    except Exception as e:
        logger.error(f"Failed to read local dataset from '{local_dataset.path}': {e}")
        raise

    return dataset
