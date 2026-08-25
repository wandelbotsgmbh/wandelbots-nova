"""Tests for wiring a dataset through `ProgramPreconditions` into `ProgramContext`."""

import datetime
from pathlib import Path
from unittest.mock import Mock

import nova
from nova import api
from nova import datasets as ds
from nova.core.nova import Nova
from nova.program import ProgramContext
from nova.types import Dataset

_TIMESTAMP = datetime.datetime(2026, 1, 1, tzinfo=datetime.timezone.utc)


def _connected_nova() -> Nova:
    nova_mock = Mock(spec=Nova)
    nova_mock.is_connected.return_value = True
    nova_mock.cell.return_value = None
    return nova_mock


def _dataset_file(tmp_path: Path, dataset_id: str = "default") -> Path:
    path = tmp_path / "dataset.json"
    response = api.models.GetDatasetResponse(
        dataset=dataset_id,
        name="Default",
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
        coordinate_systems=[],
        command_routines=[],
    )
    path.write_text(response.model_dump_json())
    return path


class TestProgramContextDataset:
    def test_defaults_to_none(self):
        ctx = ProgramContext(nova=_connected_nova(), program_id="p")

        assert ctx.dataset is None

    def test_exposes_the_dataset_it_was_built_with(self):
        dataset = Dataset(
            dataset="default", revision=1, created_at=_TIMESTAMP, updated_at=_TIMESTAMP
        )

        ctx = ProgramContext(nova=_connected_nova(), program_id="p", dataset=dataset)

        assert ctx.dataset is dataset


class TestProgramPreconditions:
    def test_accepts_a_local_request(self):
        preconditions = nova.ProgramPreconditions(
            dataset=ds.local_dataset("examples/example_dataset.json")
        )

        assert preconditions.dataset is not None
        assert preconditions.dataset.type == "local"

    def test_accepts_a_remote_request(self):
        preconditions = nova.ProgramPreconditions(dataset=ds.remote_dataset("default", revision=1))

        assert preconditions.dataset is not None
        assert preconditions.dataset.type == "remote"
        assert preconditions.dataset.revision == 1

    def test_defaults_to_no_dataset(self):
        assert nova.ProgramPreconditions().dataset is None

    def test_serializes_with_its_discriminator(self):
        """The dumped payload is what gets published to the program store."""
        preconditions = nova.ProgramPreconditions(dataset=ds.remote_dataset("default", revision=1))

        dumped = preconditions.model_dump(mode="json")

        assert dumped["dataset"] == {"dataset": "default", "revision": 1, "type": "remote"}

    def test_round_trips_through_the_discriminated_union(self):
        preconditions = nova.ProgramPreconditions(dataset=ds.remote_dataset("default"))

        restored = nova.ProgramPreconditions.model_validate(preconditions.model_dump())

        assert restored.dataset == preconditions.dataset


class TestDatasetIsLoadedIntoContext:
    async def test_program_receives_the_loaded_dataset(self, tmp_path: Path):
        path = _dataset_file(tmp_path)

        @nova.program(
            id="uses-dataset",
            preconditions=nova.ProgramPreconditions(dataset=ds.local_dataset(str(path))),
        )
        async def uses_dataset(ctx: nova.ProgramContext) -> str:
            assert ctx.dataset is not None
            return ctx.dataset.poses["pick"].dataset_pose

        assert await uses_dataset(nova=_connected_nova()) == "pick"

    async def test_loaded_dataset_is_the_convenience_type(self, tmp_path: Path):
        path = _dataset_file(tmp_path)

        @nova.program(
            id="checks-type",
            preconditions=nova.ProgramPreconditions(dataset=ds.local_dataset(str(path))),
        )
        async def checks_type(ctx: nova.ProgramContext) -> bool:
            return isinstance(ctx.dataset, Dataset)

        assert await checks_type(nova=_connected_nova()) is True

    async def test_no_dataset_leaves_context_empty(self):
        @nova.program(id="no-dataset", preconditions=nova.ProgramPreconditions())
        async def no_dataset(ctx: nova.ProgramContext) -> bool:
            return ctx.dataset is None

        assert await no_dataset(nova=_connected_nova()) is True
