from __future__ import annotations

import asyncio
import logging
from pathlib import Path
from typing import TYPE_CHECKING

from pydantic import ValidationError

from nova import api
from nova.datasets.exceptions import DatasetError, DatasetNotFoundError
from nova.datasets.types import Dataset

if TYPE_CHECKING:
    from nova.core.nova import Nova
    from nova.datasets import LoadLocalDatasetRequest, LoadRemoteDatasetRequest


logger = logging.getLogger(__name__)


def from_api_model(api_dataset: api.models.GetDatasetResponse) -> Dataset:
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


def _dataset_error(exc: api.ApiException) -> DatasetError:
    """Map a raw API-client exception onto the stable `nova.datasets` exception type.

    The exception's own message (from the response body) already names what wasn't
    found, so it's reused as-is instead of writing a second, near-duplicate message.
    """
    error_cls = (
        DatasetNotFoundError if isinstance(exc, api.exceptions.NotFoundException) else DatasetError
    )
    return error_cls(str(exc))


async def fetch(nova: Nova, dataset_request: LoadRemoteDatasetRequest) -> Dataset:
    """Fetch a dataset from the NOVA instance.

    Args:
        nova: A NOVA instance.
        dataset_request: The dataset and revision to fetch.
    """
    try:
        response = await nova.api.datasets_api.get_dataset(
            cell=nova.cell().id,
            dataset=str(dataset_request.dataset),
            revision=dataset_request.revision,
        )
    except api.ApiException as exc:
        raise _dataset_error(exc) from exc

    return from_api_model(response)


async def read(dataset_request: LoadLocalDatasetRequest, base_path: Path | None) -> Dataset:
    """Read a dataset from a local JSON file.

    Args:
        dataset_request: The local dataset file to read.
        base_path: Directory a relative `dataset_request.path` is resolved against.
            An absolute path ignores this. `None` resolves a relative path against
            the current working directory.
    """
    dataset_path = base_path / dataset_request.path if base_path else dataset_request.path
    try:
        data = await asyncio.to_thread(dataset_path.read_bytes)
        response = api.models.GetDatasetResponse.model_validate_json(data)
    except FileNotFoundError as exc:
        raise DatasetNotFoundError(str(exc)) from exc
    except (OSError, ValidationError) as exc:
        raise DatasetError(str(exc)) from exc

    return from_api_model(response)


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
    except api.ApiException as exc:
        raise _dataset_error(exc) from exc


# TODO: remove maybe or dont put in the api Model rather construct it here
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
    except api.ApiException as exc:
        raise _dataset_error(exc) from exc

    return from_api_model(response)


async def delete(nova: Nova, dataset_id: api.models.DatasetId, revision: int | None = None) -> None:
    """Delete a dataset from the NOVA instance."""
    try:
        await nova.api.datasets_api.delete_dataset(
            cell=nova.cell().id, dataset=str(dataset_id), revision=revision
        )
    except api.ApiException as exc:
        raise _dataset_error(exc) from exc


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

    try:
        return await nova.api.datasets_api.localize_dataset_frame_pose(
            cell=nova.cell().id,
            dataset=str(dataset),
            revision=revision,
            frame=str(frame),
            poses=poses,
        )
    except api.ApiException as exc:
        raise _dataset_error(exc) from exc


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

    try:
        return await nova.api.datasets_api.resolve_dataset_frame_pose(
            cell=nova.cell().id,
            dataset=str(dataset),
            revision=revision,
            frame=str(frame),
            poses=poses,
        )
    except api.ApiException as exc:
        raise _dataset_error(exc) from exc
