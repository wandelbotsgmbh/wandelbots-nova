from __future__ import annotations

import asyncio
import logging
from os import PathLike
from pathlib import Path
from typing import TYPE_CHECKING

from pydantic import ValidationError

from nova import api
from nova.config import CELL_NAME
from nova.datasets.exceptions import DatasetError, DatasetNotFoundError
from nova.datasets.types import Dataset

if TYPE_CHECKING:
    from nova.core.nova import Nova


logger = logging.getLogger(__name__)


def _dataset_error(exc: api.ApiException) -> DatasetError:
    """Map a raw API-client exception onto the stable `nova.datasets` exception type.

    The exception's own message (from the response body) already names what wasn't
    found, so it's reused as-is instead of writing a second, near-duplicate message.
    """
    error_cls = (
        DatasetNotFoundError if isinstance(exc, api.exceptions.NotFoundException) else DatasetError
    )
    return error_cls(str(exc))


async def fetch(
    nova: Nova, dataset: api.models.DatasetId, *, cell: str = CELL_NAME, revision=None
) -> Dataset:
    """Fetch a dataset from the NOVA instance.

    Args:
        nova: A NOVA instance.
        dataset: Identifier of the dataset to fetch.
        cell: The cell the dataset belongs to.
        revision: Revision to fetch. When omitted, the latest revision is used.
    """
    try:
        response = await nova.api.datasets_api.get_dataset(
            cell=cell, dataset=str(dataset), revision=revision
        )
    except api.ApiException as exc:
        raise _dataset_error(exc) from exc

    return Dataset.from_api_model(response)


async def read(path: PathLike, *, base_dir: Path | None) -> Dataset:
    """Read a dataset from a local JSON file.

    Args:
        path: The local dataset file to read.
        base_dir: Directory a relative `path` is resolved against. An absolute
            path ignores this. `None` resolves a relative path against the
            current working directory.
    """
    dataset_path = base_dir / path if base_dir else path
    try:
        data = await asyncio.to_thread(Path(dataset_path).read_bytes)
        response = api.models.GetDatasetResponse.model_validate_json(data)
    except FileNotFoundError as exc:
        raise DatasetNotFoundError(str(exc)) from exc
    except (OSError, ValidationError) as exc:
        raise DatasetError(str(exc)) from exc

    return Dataset.from_api_model(response)


async def transform_to_frame(
    nova: Nova,
    dataset: api.models.DatasetId,
    poses: list[api.models.Pose],
    frame: api.models.FrameId,
    *,
    revision: int | None = None,
    cell: str = CELL_NAME,
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
            cell=cell, dataset=str(dataset), revision=revision, frame=str(frame), poses=poses
        )
    except api.ApiException as exc:
        raise _dataset_error(exc) from exc


async def transform_to_world(
    nova: Nova,
    dataset: api.models.DatasetId,
    poses: list[api.models.Pose],
    frame: api.models.FrameId,
    *,
    revision: int | None = None,
    cell: str = CELL_NAME,
) -> list[api.models.Pose]:
    """Resolve poses from the dataset frame to world coordinates."""
    if not len(poses):
        logger.warning("No dataset poses provided, returning empty list.")
        return []

    try:
        return await nova.api.datasets_api.resolve_dataset_frame_pose(
            cell=cell, dataset=str(dataset), revision=revision, frame=str(frame), poses=poses
        )
    except api.ApiException as exc:
        raise _dataset_error(exc) from exc
