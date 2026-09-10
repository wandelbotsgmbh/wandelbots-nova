"""Unit tests for the `nova.datasets.Dataset` convenience wrapper and load requests."""

import datetime
from pathlib import Path

import pytest
from pydantic import ValidationError

import nova
from nova import api
from nova import datasets as ds
from nova.datasets import Dataset, LoadLocalDatasetRequest

_TIMESTAMP = datetime.datetime(2026, 1, 1, tzinfo=datetime.timezone.utc)


def _pose(dataset_pose: str, dataset: str = "default") -> api.models.DatasetPose:
    return api.models.DatasetPose(
        pose=api.models.Pose(position=[1, 2, 3], orientation=[0, 0, 0]),
        dataset_pose=dataset_pose,
        dataset=dataset,
    )


def _dataset(**overrides) -> Dataset:
    defaults = dict(
        dataset="default", name="Default", revision=2, created_at=_TIMESTAMP, updated_at=_TIMESTAMP
    )
    defaults.update(overrides)
    return Dataset(**defaults)


class TestDataset:
    def test_carries_over_scalar_fields(self):
        dataset = _dataset()

        assert dataset.dataset == "default"
        assert dataset.name == "Default"
        assert dataset.revision == 2
        assert dataset.created_at == _TIMESTAMP

    def test_nested_collections_default_to_empty_dicts(self):
        dataset = _dataset()

        assert dataset.poses == {}
        assert dataset.command_routines == {}
        assert dataset.frames == {}

    def test_accepts_poses_and_frames_keyed_by_their_identifier(self):
        dataset = _dataset(
            poses={"pick": _pose("pick"), "place": _pose("place")},
            frames={
                "fixture": api.models.DatasetFrame(
                    frame="fixture",
                    pose=api.models.Pose(position=[0, 0, 0], orientation=[0, 0, 0]),
                    dataset="default",
                )
            },
        )

        assert set(dataset.poses) == {"pick", "place"}
        assert dataset.poses["pick"].dataset_pose == "pick"
        assert set(dataset.frames) == {"fixture"}


class TestLoadRequests:
    def test_remote_request_defaults(self):
        request = ds.remote_dataset("default")

        assert request.type == "remote"
        assert request.dataset == "default"
        assert request.revision is None

    def test_local_request_keeps_a_relative_path_as_written(self):
        """Resolution is deferred to load time, so the request stays machine-portable."""
        request = ds.local_dataset("example_dataset.json")

        assert request.type == "local"
        assert request.path == Path("example_dataset.json")

    def test_local_request_rejects_an_absolute_path(self, tmp_path: Path):
        """An absolute path defeats portability across machines, so it's rejected outright."""
        path = tmp_path / "dataset.json"

        with pytest.raises(ValueError, match="must be relative"):
            ds.local_dataset(str(path))

    def test_relative_local_request_survives_a_serialization_round_trip(self):
        """novax publishes preconditions as JSON - a relative path must stay relative."""
        preconditions = nova.ProgramPreconditions(dataset=ds.local_dataset("example_dataset.json"))

        dumped = preconditions.model_dump(mode="json")
        restored = nova.ProgramPreconditions.model_validate(dumped)

        assert dumped["dataset"]["path"] == "example_dataset.json"
        assert isinstance(restored.dataset, LoadLocalDatasetRequest)
        assert Path(restored.dataset.path) == Path("example_dataset.json")

    def test_remote_dataset_id_must_match_the_id_pattern(self):
        with pytest.raises(ValidationError):
            ds.remote_dataset("9bad_id")

    def test_requests_are_frozen(self):
        request = ds.remote_dataset("default")

        with pytest.raises(Exception):
            request.dataset = "other"  # ty: ignore
