"""Resolve a dataset's poses and look up its command routines.

A teaching dataset (``api.models.GetDatasetResponse``) stores each pose relative to a named
coordinate system, and coordinate systems can themselves be nested (``parent``) to keep taught
poses portable across e.g. re-fixturing a workstation. :func:`resolve_dataset_poses` walks
those chains once and returns a flat ``dataset_pose id -> Pose`` mapping, ready to pass as the
``pose_resolver`` argument of :func:`nova.command_routines.actions_from_command_routine` so a
routine's ``LocalPoseReference`` targets can be resolved.

A dataset can hold several command routines; :func:`get_command_routine` picks out the one
with a given id.
"""

from __future__ import annotations

from nova import api
from nova.types import Pose

_IDENTITY_POSE = Pose(position=(0, 0, 0), orientation=(0, 0, 0))


def resolve_dataset_poses(dataset: api.models.GetDatasetResponse) -> dict[str, Pose]:
    """Return a ``{dataset_pose_id: Pose}`` mapping with every pose resolved to world frame.

    Raises:
        ValueError: if a pose or coordinate system references an unknown coordinate system id,
            or if the coordinate systems contain a cycle.
    """
    coordinate_systems = {cs.coordinate_system: cs for cs in dataset.coordinate_systems}
    resolved_coordinate_systems: dict[str, Pose] = {}

    def resolve_coordinate_system(coordinate_system_id: str, chain: tuple[str, ...]) -> Pose:
        if coordinate_system_id in resolved_coordinate_systems:
            return resolved_coordinate_systems[coordinate_system_id]
        if coordinate_system_id in chain:
            cycle = " -> ".join((*chain, coordinate_system_id))
            raise ValueError(f"Cyclic coordinate system reference: {cycle}")
        try:
            coordinate_system = coordinate_systems[coordinate_system_id]
        except KeyError:
            raise ValueError(f"Unknown coordinate system {coordinate_system_id!r}") from None

        own_pose = Pose.from_api_model(coordinate_system.pose)
        resolved = (
            own_pose
            if coordinate_system.parent is None
            else resolve_coordinate_system(coordinate_system.parent, (*chain, coordinate_system_id))
            @ own_pose
        )
        resolved_coordinate_systems[coordinate_system_id] = resolved
        return resolved

    world_poses: dict[str, Pose] = {}
    for pose in dataset.poses:
        local_pose = Pose.from_api_model(pose.pose)
        base_pose = (
            _IDENTITY_POSE
            if pose.coordinate_system is None
            else resolve_coordinate_system(pose.coordinate_system, ())
        )
        resolved_pose = base_pose @ local_pose
        if pose.kinematic_configuration is not None:
            resolved_pose = resolved_pose.model_copy(
                update={"kinematic_configuration": pose.kinematic_configuration}
            )
        world_poses[pose.dataset_pose] = resolved_pose

    return world_poses


def get_command_routine(
    dataset: api.models.GetDatasetResponse, command_routine_id: str
) -> api.models.CommandRoutine:
    """Return the command routine with id ``command_routine_id`` from ``dataset``.

    Raises:
        ValueError: if no command routine with that id exists in the dataset.
    """
    for routine in dataset.command_routines:
        if routine.command_routine == command_routine_id:
            return routine
    raise ValueError(
        f"No command routine {command_routine_id!r} in dataset {dataset.dataset!r}. "
        f"Available: {[routine.command_routine for routine in dataset.command_routines]}"
    )
