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
from nova.types import Dataset

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
    """A connected Nova whose `datasets_api` methods return the given values."""
    nova = Mock(spec=Nova)
    nova.is_connected.return_value = True
    nova.cell.return_value = Mock(id="cell")
    datasets_api = Mock()
    for name, value in api_returns.items():
        setattr(datasets_api, name, AsyncMock(return_value=value))
    nova.api = Mock(datasets_api=datasets_api)
    return nova


class TestListDatasets:
    async def test_passes_no_filters_by_default(self):
        nova = _nova_mock(get_datasets=[])
        assert await ds.list_datasets(nova) == []
        nova.api.datasets_api.get_datasets.assert_awaited_once_with(
            cell="cell", dataset=None, latest_only=None
        )

    async def test_forwards_dataset_and_latest_only_filters(self):
        nova = _nova_mock(get_datasets=[])
        await ds.list_datasets(nova, dataset_id="default", latest_only=True)
        nova.api.datasets_api.get_datasets.assert_awaited_once_with(
            cell="cell", dataset="default", latest_only=True
        )

    async def test_requires_connected_nova(self):
        nova = _nova_mock(get_datasets=[])
        nova.is_connected.return_value = False
        with pytest.raises(AssertionError):
            await ds.list_datasets(nova)


class TestCreateAndDeleteDataset:
    async def test_create_forwards_the_request_unchanged(self):
        response = _dataset_response("new-set")
        nova = _nova_mock(create_dataset=response)
        create_request = api.models.CreateDatasetRequest(dataset="new-set", name="New set")

        result = await ds.create_dataset(nova, create_request)

        assert result.dataset == response.dataset
        nova.api.datasets_api.create_dataset.assert_awaited_once_with(
            cell="cell", create_dataset_request=create_request
        )

    async def test_create_returns_the_convenience_type(self):
        nova = _nova_mock(create_dataset=_dataset_response("new-set"))

        result = await ds.create_dataset(nova, api.models.CreateDatasetRequest(dataset="new-set"))

        assert isinstance(result, Dataset)
        assert set(result.poses) == {"pick"}

    async def test_delete_forwards_revision(self):
        nova = _nova_mock(delete_dataset=None)
        await ds.delete_dataset(nova, "new-set", revision=3)
        nova.api.datasets_api.delete_dataset.assert_awaited_once_with(
            cell="cell", dataset="new-set", revision=3
        )


class TestLoadDataset:
    async def test_remote_request_is_fetched_from_the_instance(self):
        nova = _nova_mock(get_dataset=_dataset_response("default"))

        result = await ds.load_dataset(nova, ds.remote_dataset("default", revision=2))

        assert result.dataset == "default"
        nova.api.datasets_api.get_dataset.assert_awaited_once_with(
            cell="cell", dataset="default", revision=2
        )

    async def test_local_request_is_read_from_disk(self, tmp_path: Path):
        path = tmp_path / "dataset.json"
        path.write_text(_dataset_response("source-set").model_dump_json())

        result = await ds.load_dataset(Mock(spec=Nova), ds.local_dataset(str(path)))

        assert result.dataset == "source-set"
        assert [pose.dataset for pose in result.poses.values()] == ["source-set"]


class TestReadLocalDataset:
    async def test_reads_and_parses_json(self, tmp_path: Path):
        path = tmp_path / "dataset.json"
        path.write_text(_dataset_response("source-set").model_dump_json())

        result = await ds.read_local_dataset(ds.local_dataset(str(path)))

        assert result.dataset == "source-set"
        assert len(result.poses) == 1

    async def test_missing_file_raises(self, tmp_path: Path):
        with pytest.raises(OSError):
            await ds.read_local_dataset(ds.local_dataset(str(tmp_path / "nope.json")))

    async def test_malformed_json_raises(self, tmp_path: Path):
        path = tmp_path / "dataset.json"
        path.write_text(json.dumps({"dataset": "source-set"}))

        with pytest.raises(ValidationError):
            await ds.read_local_dataset(ds.local_dataset(str(path)))

    async def test_relative_path_is_resolved_against_relative_to(self, tmp_path: Path):
        (tmp_path / "dataset.json").write_text(_dataset_response("source-set").model_dump_json())

        result = await ds.read_local_dataset(ds.local_dataset("dataset.json"), relative_to=tmp_path)

        assert result.dataset == "source-set"

    async def test_relative_path_ignores_the_working_directory(
        self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
    ):
        """The anchor wins even when a same-named file sits in the working directory."""
        (tmp_path / "dataset.json").write_text(_dataset_response("anchored").model_dump_json())
        decoy = tmp_path / "decoy"
        decoy.mkdir()
        (decoy / "dataset.json").write_text(_dataset_response("from-cwd").model_dump_json())
        monkeypatch.chdir(decoy)

        result = await ds.read_local_dataset(ds.local_dataset("dataset.json"), relative_to=tmp_path)

        assert result.dataset == "anchored"

    async def test_relative_path_without_an_anchor_raises(self):
        with pytest.raises(ValueError, match="Cannot resolve the relative dataset path"):
            await ds.read_local_dataset(ds.local_dataset("dataset.json"))
