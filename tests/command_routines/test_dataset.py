import pytest

from nova import api
from nova.command_routines import (
    command_routine,
    get_command_routine,
    move_joint_ptp,
    resolve_dataset_poses,
)
from nova.types import Pose

_NOW = "2026-05-20T07:20:10.411751Z"


def _dataset(
    coordinate_systems: tuple[api.models.DatasetCoordinateSystem, ...] = (),
    poses: tuple[api.models.DatasetPose, ...] = (),
    command_routines: tuple[api.models.CommandRoutine, ...] = (),
) -> api.models.GetDatasetResponse:
    return api.models.GetDatasetResponse(
        dataset="test-dataset",
        revision=1,
        created_at=_NOW,
        updated_at=_NOW,
        coordinate_systems=list(coordinate_systems),
        poses=list(poses),
        command_routines=list(command_routines),
    )


def _coordinate_system(
    coordinate_system: str, position, parent: str | None = None
) -> api.models.DatasetCoordinateSystem:
    return api.models.DatasetCoordinateSystem(
        coordinate_system=coordinate_system,
        dataset="test-dataset",
        parent=parent,
        pose=api.models.Pose(position=position, orientation=(0, 0, 0)),
    )


def _pose(
    dataset_pose: str,
    position,
    *,
    coordinate_system: str | None = None,
    kinematic_configuration: api.models.KinematicConfiguration | None = None,
) -> api.models.DatasetPose:
    return api.models.DatasetPose(
        dataset_pose=dataset_pose,
        dataset="test-dataset",
        coordinate_system=coordinate_system,
        pose=api.models.Pose(position=position, orientation=(0, 0, 0)),
        kinematic_configuration=kinematic_configuration,
    )


class TestResolveDatasetPoses:
    def test_resolves_pose_relative_to_a_root_coordinate_system(self):
        dataset = _dataset(
            coordinate_systems=[_coordinate_system("station", (900, 0, 0))],
            poses=[_pose("p1", (-150, -250, 85), coordinate_system="station")],
        )
        poses = resolve_dataset_poses(dataset)
        assert poses["p1"] == Pose((750, -250, 85, 0, 0, 0))

    def test_resolves_pose_through_a_nested_coordinate_system_chain(self):
        dataset = _dataset(
            coordinate_systems=[
                _coordinate_system("world", (0, 0, 140)),
                _coordinate_system("station", (900, 0, 0), parent="world"),
            ],
            poses=[_pose("p1", (-150, -250, 85), coordinate_system="station")],
        )
        poses = resolve_dataset_poses(dataset)
        assert poses["p1"] == Pose((750, -250, 225, 0, 0, 0))

    def test_pose_without_coordinate_system_is_used_as_is(self):
        dataset = _dataset(poses=[_pose("p1", (1, 2, 3))])
        poses = resolve_dataset_poses(dataset)
        assert poses["p1"] == Pose((1, 2, 3, 0, 0, 0))

    def test_kinematic_configuration_is_preserved(self):
        kc = api.models.KinematicConfiguration(
            kinematic_branch=api.models.KinematicBranch(
                shoulder_branch="FRONT", elbow_branch="DOWN", wrist_branch="NO_FLIP"
            )
        )
        dataset = _dataset(poses=[_pose("p1", (1, 2, 3), kinematic_configuration=kc)])
        poses = resolve_dataset_poses(dataset)
        assert poses["p1"].kinematic_configuration == kc

    def test_unknown_coordinate_system_raises(self):
        dataset = _dataset(poses=[_pose("p1", (1, 2, 3), coordinate_system="missing")])
        with pytest.raises(ValueError, match="Unknown coordinate system"):
            resolve_dataset_poses(dataset)

    def test_unknown_parent_coordinate_system_raises(self):
        dataset = _dataset(
            coordinate_systems=[_coordinate_system("station", (0, 0, 0), parent="missing")],
            poses=[_pose("p1", (1, 2, 3), coordinate_system="station")],
        )
        with pytest.raises(ValueError, match="Unknown coordinate system"):
            resolve_dataset_poses(dataset)

    def test_cyclic_coordinate_systems_raise(self):
        dataset = _dataset(
            coordinate_systems=[
                _coordinate_system("a", (0, 0, 0), parent="b"),
                _coordinate_system("b", (0, 0, 0), parent="a"),
            ],
            poses=[_pose("p1", (1, 2, 3), coordinate_system="a")],
        )
        with pytest.raises(ValueError, match="Cyclic coordinate system reference"):
            resolve_dataset_poses(dataset)

    def test_multiple_poses_share_a_coordinate_system(self):
        dataset = _dataset(
            coordinate_systems=[_coordinate_system("station", (900, 0, 0))],
            poses=[
                _pose("p1", (0, 0, 0), coordinate_system="station"),
                _pose("p2", (100, 0, 0), coordinate_system="station"),
            ],
        )
        poses = resolve_dataset_poses(dataset)
        assert poses["p1"] == Pose((900, 0, 0, 0, 0, 0))
        assert poses["p2"] == Pose((1000, 0, 0, 0, 0, 0))


class TestGetCommandRoutine:
    def test_returns_the_matching_routine(self):
        first = command_routine("first", commands=[move_joint_ptp([0.0] * 6)])
        second = command_routine("second", commands=[move_joint_ptp([0.1] * 6)])
        dataset = _dataset(command_routines=[first, second])

        assert get_command_routine(dataset, "second") is second

    def test_unknown_id_raises_with_available_ids_listed(self):
        routine = command_routine("only-one", commands=[move_joint_ptp([0.0] * 6)])
        dataset = _dataset(command_routines=[routine])

        with pytest.raises(ValueError, match="only-one"):
            get_command_routine(dataset, "missing")
