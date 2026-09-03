from __future__ import annotations

import asyncio
import logging
from typing import TYPE_CHECKING

from nova import api
from nova.datasets.types import Dataset, LoadDatasetRequest
from nova.datasets.utils import executing_program_dir

if TYPE_CHECKING:
    from nova.core.nova import Nova

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


async def _load_dataset_in_context(nova: Nova, dataset_request: LoadDatasetRequest) -> Dataset:
    """Load a dataset from the NOVA instance or from a local file.

    Args:
        nova: A NOVA instance.
        dataset_request: The dataset and source to load.
    """

    if dataset_request.type == "remote":
        try:
            response = await nova.api.datasets_api.get_dataset(
                cell=nova.cell().id,
                dataset=str(dataset_request.dataset),
                revision=dataset_request.revision,
            )
        except Exception:
            logger.exception(f"Failed to fetch dataset '{dataset_request.dataset}'")
            raise
    else:
        program_dir = executing_program_dir()
        if program_dir is None:
            raise RuntimeError("load_dataset() must be called while a @nova.program is executing.")
        path = program_dir / dataset_request.path
        try:
            data = await asyncio.to_thread(path.read_text)
            response = api.models.GetDatasetResponse.model_validate_json(data)
        except Exception:
            logger.exception(f"Failed to read local dataset from '{path}'")
            raise

    return _dataset_from_api(response)


async def list_all(
    nova: Nova, dataset_id: api.models.DatasetId | None = None, latest_only: bool | None = None
) -> list[api.models.Dataset]:
    """List all the datasets available in the cell on the NOVA instance.

    Every revision is returned as its own entry unless the result is narrowed down.

    Args:
        nova: A NOVA instance.
        dataset_id: Restrict the result to all revisions of this dataset.
        latest_only: When True, return only the latest revision of each dataset.
    """
    try:
        return await nova.api.datasets_api.get_datasets(
            cell=nova.cell().id,
            dataset=str(dataset_id) if dataset_id is not None else None,
            latest_only=latest_only,
        )
    except Exception:
        logger.exception("Failed to list datasets")
        raise


async def create(nova: Nova, create_request: api.models.CreateDatasetRequest) -> Dataset:
    """Create a dataset together with its poses, frames and command routines.

    Args:
        nova: A NOVA instance.
        create_request: The dataset to create. Server-managed fields are assigned by
            the NOVA instance and must not be part of the request.
    """
    try:
        response = await nova.api.datasets_api.create_dataset(
            cell=nova.cell().id, create_dataset_request=create_request
        )
        return _dataset_from_api(response)
    except Exception:
        logger.exception(f"Failed to create dataset '{create_request.dataset}'")
        raise


async def delete(nova: Nova, dataset_id: api.models.DatasetId, revision: int | None = None) -> None:
    """Delete a dataset from the NOVA instance."""
    try:
        await nova.api.datasets_api.delete_dataset(
            cell=nova.cell().id, dataset=str(dataset_id), revision=revision
        )
    except Exception:
        logger.exception(f"Failed to delete dataset '{dataset_id}'")
        raise


async def transform_to_frame(
    nova: Nova,
    poses: list[api.models.Pose],
    frame: api.models.FrameId,
    dataset: api.models.DatasetId,
    revision: int | None = None,
) -> list[api.models.Pose]:
    """Localize a list of poses that are expressed in the `world` frame into the
    given dataset frame.

    Args:
        nova: A NOVA instance.
        poses: The poses to localize, expressed in the `world` frame.
        frame: The dataset frame to localize the poses into.
        dataset: The dataset that owns the frame.
        revision: The dataset revision to use. Defaults to the latest revision.
    """

    if not len(poses):
        logger.warning("No dataset poses provided, returning empty list.")
        return []

    return await nova.api.datasets_api.localize_dataset_frame_pose(
        cell=nova.cell().id, dataset=str(dataset), revision=revision, frame=str(frame), poses=poses
    )


async def transform_to_world(
    nova: Nova,
    poses: list[api.models.Pose],
    frame: api.models.FrameId,
    dataset: api.models.DatasetId,
    revision: int | None = None,
) -> list[api.models.Pose]:
    """Resolve poses from the dataset frame to world coordinates."""
    if not len(poses):
        logger.warning("No dataset poses provided, returning empty list.")
        return []

    return await nova.api.datasets_api.resolve_dataset_frame_pose(
        cell=nova.cell().id, dataset=str(dataset), revision=revision, frame=str(frame), poses=poses
    )
