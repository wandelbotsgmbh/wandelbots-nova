"""Unit tests for resolving dataset frames to world coordinates."""

import datetime

import pytest

from nova import api
from nova.actions import cartesian_ptp
from nova.datasets import Dataset, DatasetFrame, FrameResolutionError
from nova.types import Pose

_TIMESTAMP = datetime.datetime(2026, 1, 1, tzinfo=datetime.timezone.utc)


def _api_pose(pose: Pose, dataset_pose: str, frame: str | None = None) -> api.models.DatasetPose:
    return api.models.DatasetPose(
        pose=pose.to_api_model(), dataset_pose=dataset_pose, dataset="d", frame=frame
    )


def _api_frame(
    pose: Pose, frame: str, reference_frame: str | None = None
) -> api.models.DatasetFrame:
    return api.models.DatasetFrame(
        pose=pose.to_api_model(), frame=frame, reference_frame=reference_frame, dataset="d"
    )


def _dataset(
    poses: list[api.models.DatasetPose] | None = None,
    frames: list[api.models.DatasetFrame] | None = None,
) -> Dataset:
    return Dataset.from_api_model(
        api.models.GetDatasetResponse(
            dataset="d",
            name="D",
            revision=1,
            created_at=_TIMESTAMP,
            updated_at=_TIMESTAMP,
            poses=poses or [],
            frames=frames or [],
            command_routines=[],
        )
    )


TABLE = Pose((100, -200, 800, 0, 0, 0))
FIXTURE = Pose((1000, 1000, 500, 0, 0, 0))
TOOL = Pose((0, 1500, 0, 0, 0, 0))
SLOT = Pose((500, -400, 0, 0, 0, 0))


@pytest.fixture
def nested() -> Dataset:
    """A three level frame chain: table <- fixture <- tool, mirroring the seeded dataset."""
    return _dataset(
        poses=[
            _api_pose(Pose((1, 2, 3, 0, 0, 0)), "home"),
            _api_pose(SLOT, "table-origin", frame="table"),
            _api_pose(SLOT, "fixture-slot-a", frame="fixture"),
            _api_pose(SLOT, "tool-tip", frame="tool"),
        ],
        frames=[
            _api_frame(TABLE, "table"),
            _api_frame(FIXTURE, "fixture", reference_frame="table"),
            _api_frame(TOOL, "tool", reference_frame="fixture"),
        ],
    )


class TestResolution:
    def test_world_frame_pose_is_returned_unchanged(self, nested: Dataset):
        home = nested.poses["home"]

        assert home.as_world() == home.pose

    def test_single_level_frame(self, nested: Dataset):
        pose = nested.poses["table-origin"]

        assert pose.as_world() == TABLE @ SLOT

    def test_nested_frames_compose_the_whole_chain(self, nested: Dataset):
        pose = nested.poses["tool-tip"]

        assert pose.as_world() == TABLE @ FIXTURE @ TOOL @ SLOT

    def test_frame_resolves_to_its_own_world_transform(self, nested: Dataset):
        assert nested.frames["table"].as_world() == TABLE
        assert nested.frames["fixture"].as_world() == TABLE @ FIXTURE
        assert nested.frames["tool"].as_world() == TABLE @ FIXTURE @ TOOL


class TestDynamicFrames:
    """A frame measured at runtime replaces the taught one and everything below follows."""

    def test_replacing_a_frame_moves_the_poses_on_it(self, nested: Dataset):
        measured = Pose((10, 20, 30, 0, 0, 0))

        # Resolve first, so a stale cache would be caught.
        assert nested.poses["fixture-slot-a"].as_world() == TABLE @ FIXTURE @ SLOT

        nested.set_frame("fixture", measured, reference_frame="table")

        assert nested.poses["fixture-slot-a"].as_world() == TABLE @ measured @ SLOT

    def test_a_replaced_frame_resolves_itself(self, nested: Dataset):
        measured = Pose((10, 20, 30, 0, 0, 0))

        frame = nested.set_frame("fixture", measured, reference_frame="table")

        assert frame.as_world() == TABLE @ measured
        assert nested.frames["fixture"].as_world() == TABLE @ measured

    def test_replacing_a_frame_moves_the_frames_below_it(self, nested: Dataset):
        measured = Pose((10, 20, 30, 0, 0, 0))

        nested.set_frame("fixture", measured, reference_frame="table")

        assert nested.frames["tool"].as_world() == TABLE @ measured @ TOOL

    def test_a_frame_can_be_rebound_to_world(self, nested: Dataset):
        measured = Pose((10, 20, 30, 0, 0, 0))

        nested.set_frame("fixture", measured)

        assert nested.frames["fixture"].as_world() == measured
        assert nested.poses["fixture-slot-a"].as_world() == measured @ SLOT

    def test_a_new_frame_can_be_added(self, nested: Dataset):
        nested.set_frame("pallet", Pose((1, 1, 1, 0, 0, 0)), reference_frame="table")

        assert nested.frames["pallet"].as_world() == TABLE @ Pose((1, 1, 1, 0, 0, 0))

    def test_replacing_the_whole_frame_dict_is_picked_up(self, nested: Dataset):
        """The tree reads frames off the dataset, so it cannot hold on to a stale dict."""
        measured = Pose((10, 20, 30, 0, 0, 0))
        assert nested.poses["fixture-slot-a"].as_world() == TABLE @ FIXTURE @ SLOT

        nested.frames = {
            "table": nested.frames["table"],
            "fixture": DatasetFrame(
                frame="fixture", reference_frame="table", dataset="d", pose=measured
            ),
        }

        assert nested.poses["fixture-slot-a"].as_world() == TABLE @ measured @ SLOT


class TestRoundTrip:
    def test_localizing_a_world_pose_returns_the_taught_pose(self, nested: Dataset):
        pose = nested.poses["fixture-slot-a"]
        fixture = nested.frames["fixture"].as_world()

        assert ~fixture @ pose.as_world() == pose.pose

    def test_a_pose_can_be_expressed_in_another_frame(self, nested: Dataset):
        """A pose taught in `fixture`, re-expressed in `table` coordinates."""
        pose = nested.poses["fixture-slot-a"]
        table = nested.frames["table"].as_world()

        assert ~table @ pose.as_world() == FIXTURE @ SLOT

    def test_a_frame_origin_can_be_overridden_at_runtime(self, nested: Dataset):
        """The vision case: the frame origin is measured, the slot stays where it was taught."""
        measured = Pose((10, 20, 30, 0, 0, 0))

        assert measured @ nested.poses["fixture-slot-a"].pose == measured @ SLOT


class TestUnresolvableFrames:
    def test_dangling_pose_frame_fails_only_on_that_pose(self):
        dataset = _dataset(
            poses=[_api_pose(SLOT, "broken", frame="nope"), _api_pose(SLOT, "good", frame="table")],
            frames=[_api_frame(TABLE, "table")],
        )

        with pytest.raises(FrameResolutionError, match="'nope' is not defined"):
            dataset.poses["broken"].as_world()

        assert dataset.poses["good"].as_world() == TABLE @ SLOT

    def test_dangling_reference_frame(self):
        dataset = _dataset(frames=[_api_frame(FIXTURE, "fixture", reference_frame="nope")])

        with pytest.raises(FrameResolutionError, match="'nope' is not defined"):
            dataset.frames["fixture"].as_world()

    def test_cycle_reports_the_chain(self):
        dataset = _dataset(
            frames=[
                _api_frame(TABLE, "a", reference_frame="b"),
                _api_frame(FIXTURE, "b", reference_frame="a"),
            ]
        )

        with pytest.raises(FrameResolutionError, match="cycle") as excinfo:
            dataset.frames["a"].as_world()

        assert excinfo.value.frame == "a"
        assert excinfo.value.chain == ["a", "b", "a"]
        # The chain broke, so the rendered chain must not claim to reach world.
        assert "chain: a -> b -> a)" in str(excinfo.value)

    def test_self_referencing_frame(self):
        dataset = _dataset(frames=[_api_frame(TABLE, "a", reference_frame="a")])

        with pytest.raises(FrameResolutionError) as excinfo:
            dataset.frames["a"].as_world()

        assert excinfo.value.chain == ["a", "a"]

    def test_detached_entry_reports_that_it_has_no_dataset(self):
        """A pose that was round-tripped through serialization has lost its frame tree."""
        pose = _dataset(
            poses=[_api_pose(SLOT, "slot", frame="table")], frames=[_api_frame(TABLE, "table")]
        ).poses["slot"]
        detached = type(pose).model_validate(pose.model_dump())

        with pytest.raises(FrameResolutionError, match="not attached to a dataset"):
            detached.as_world()


class TestMotionTargets:
    def test_a_world_pose_can_be_used_directly(self, nested: Dataset):
        home = nested.poses["home"]

        assert cartesian_ptp(home).target == home.pose

    def test_a_frame_relative_pose_is_rejected(self, nested: Dataset):
        with pytest.raises(ValueError, match="expressed in frame 'fixture'"):
            cartesian_ptp(nested.poses["fixture-slot-a"])

    def test_a_resolved_pose_is_accepted(self, nested: Dataset):
        pose = nested.poses["fixture-slot-a"]

        assert cartesian_ptp(pose.as_world()).target == TABLE @ FIXTURE @ SLOT
