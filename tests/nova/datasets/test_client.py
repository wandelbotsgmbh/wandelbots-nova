"""Unit tests for the dataset convenience helpers in `nova.datasets`."""

import datetime
import json
from pathlib import Path
from unittest.mock import AsyncMock, Mock

import pytest
from pydantic import ValidationError

from nova import api
from nova import datasets as ds
from nova.core.nova import Nova
from nova.datasets import Dataset, LoadLocalDatasetRequest

_TIMESTAMP = datetime.datetime(2026, 1, 1, tzinfo=datetime.timezone.utc)


def _dataset_response(dataset_id: str = "source-set") -> api.models.GetDatasetResponse:
    """Build a dataset response with one entry in each nested collection."""
    return api.models.GetDatasetResponse(
        dataset=dataset_id,
        name="Source set",
        revision=1,
        created_at=_TIMESTAMP,
        updated_at=_TIMESTAMP,
        poses=[
            api.models.DatasetPose(
                pose=api.models.Pose(position=[1, 2, 3], orientation=[0, 0, 0]),
                dataset_pose="pick",
                dataset=dataset_id,
            )
        ],
        frames=[
            api.models.DatasetFrame(
                frame="fixture",
                pose=api.models.Pose(position=[0, 0, 0], orientation=[0, 0, 0]),
                dataset=dataset_id,
            )
        ],
        command_routines=[],
    )


def _nova_mock(**api_returns) -> Nova:
    """A Nova whose `datasets_api` methods return the given values.

    Deliberately leaves `is_connected` unstubbed: dataset operations go over plain HTTP
    through `ApiGateway`, so none of them may depend on the NATS connection state.
    """
    nova = Mock(spec=Nova)
    nova.cell.return_value = Mock(id="cell")
    datasets_api = Mock()
    for name, value in api_returns.items():
        setattr(datasets_api, name, AsyncMock(return_value=value))
    nova.api = Mock(datasets_api=datasets_api)
    return nova


class TestAllDatasets:
    async def test_passes_no_filters_by_default(self):
        nova = _nova_mock(get_datasets=[])
        assert await ds.list_all(nova) == []
        nova.api.datasets_api.get_datasets.assert_awaited_once_with(
            cell="cell", dataset=None, latest_only=None
        )

    async def test_forwards_dataset_and_latest_only_filters(self):
        nova = _nova_mock(get_datasets=[])
        await ds.list_all(nova, dataset_id="default", latest_only=True)
        nova.api.datasets_api.get_datasets.assert_awaited_once_with(
            cell="cell", dataset="default", latest_only=True
        )


class TestCreateAndDeleteDataset:
    async def test_create_forwards_the_request_unchanged(self):
        response = _dataset_response("new-set")
        nova = _nova_mock(create_dataset=response)
        create_request = api.models.CreateDatasetRequest(dataset="new-set", name="New set")

        result = await ds.create(nova, create_request)

        assert result.dataset == response.dataset
        nova.api.datasets_api.create_dataset.assert_awaited_once_with(
            cell="cell", create_dataset_request=create_request
        )

    async def test_create_returns_the_convenience_type(self):
        nova = _nova_mock(create_dataset=_dataset_response("new-set"))

        result = await ds.create(nova, api.models.CreateDatasetRequest(dataset="new-set"))

        assert isinstance(result, Dataset)
        assert set(result.poses) == {"pick"}

    async def test_delete_forwards_revision(self):
        nova = _nova_mock(delete_dataset=None)
        await ds.delete(nova, "new-set", revision=3)
        nova.api.datasets_api.delete_dataset.assert_awaited_once_with(
            cell="cell", dataset="new-set", revision=3
        )


class TestFetchDataset:
    async def test_fetches_from_the_instance(self):
        nova_mock = _nova_mock(get_dataset=_dataset_response("default"))

        result = await ds.fetch(nova_mock, ds.remote_dataset("default", revision=2))

        assert result.dataset == "default"
        nova_mock.api.datasets_api.get_dataset.assert_awaited_once_with(
            cell="cell", dataset="default", revision=2
        )


class TestReadDataset:
    async def test_reads_from_disk(self, tmp_path: Path):
        path = tmp_path / "dataset.json"
        path.write_text(_dataset_response("source-set").model_dump_json())

        result = await ds.read(LoadLocalDatasetRequest(path=path), base_path=None)

        assert result.dataset == "source-set"
        assert [pose.dataset for pose in result.poses.values()] == ["source-set"]

    async def test_resolves_a_relative_path_against_base_path(self, tmp_path: Path):
        path = tmp_path / "dataset.json"
        path.write_text(_dataset_response("source-set").model_dump_json())

        result = await ds.read(
            LoadLocalDatasetRequest(path=Path("dataset.json")), base_path=tmp_path
        )

        assert result.dataset == "source-set"

    async def test_missing_file_raises(self, tmp_path: Path):
        with pytest.raises(OSError):
            await ds.read(LoadLocalDatasetRequest(path=tmp_path / "nope.json"), base_path=None)

    async def test_malformed_json_raises(self, tmp_path: Path):
        path = tmp_path / "dataset.json"
        path.write_text(json.dumps({"dataset": "source-set"}))

        with pytest.raises(ValidationError):
            await ds.read(LoadLocalDatasetRequest(path=path), base_path=None)
