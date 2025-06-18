import math

import numpy as np
from geometricalgebra import cga3d
from geometricalgebra.cga import project_to_flat

from nova.types import Pose, Vector3d
from wandelscript.metamodel import register_builtin_func
from wandelscript.utils.pose import pose_to_versor, versor_to_pose


@register_builtin_func()
def interpolate(a: Pose, b: Pose, param: float | list[float]) -> Pose | list[Pose]:
    """Interpolate between two poses"""
    va = pose_to_versor(a)
    vb = pose_to_versor(b)
    vc = (vb & va.inverse()).motor_interpolation(param) & va
    if isinstance(param, list):
        return [versor_to_pose(i) for i in vc]
    return versor_to_pose(vc)


@register_builtin_func()
def distance(a: Pose | Vector3d, b: Pose | Vector3d) -> float:
    """Distance in [mm] between two poses or positions

    Args:
        a: The first pose or position
        b: The second pose or position

    Returns:
        The distance between the two poses or positions

    """
    a = a if isinstance(a, Vector3d) else a.position
    b = b if isinstance(b, Vector3d) else b.position
    return math.sqrt((-2 * cga3d.Vector.from_euclid(a) | cga3d.Vector.from_euclid(b)).to_scalar())


@register_builtin_func()
def to_position(pose: Pose) -> Vector3d:
    """Extract the position from a pose."""
    return pose.position


@register_builtin_func()
def to_orientation(pose: Pose) -> Vector3d:
    """Extract the orientation from a pose."""
    return pose.orientation


@register_builtin_func()
def find_edge_from_4_poses(
    left_start: Pose, left_end: Pose, right_start: Pose, right_end: Pose
) -> tuple[Pose, Pose]:
    """Given 6 points, two poses describing the edge are returned

    Args:
        left_start: pose whose xy-plane is parallel to the left surface and the origin is close to start position
        left_end: pose whose xy-plane is parallel to the left surface and the origin is close to end position
        right_start: pose whose xy-plane is parallel to the right surface and the origin is close to start position
        right_end: pose whose xy-plane is parallel to the right surface and the origin is close to end position

    Returns:
        start_pose: a pose with position at the start and an orientation normal to the left plane
        end_pose: a pose with position at the end and an orientation normal to the right plane
    """
    right_start_versor = pose_to_versor(right_start)
    right_end_versor = pose_to_versor(right_end)
    left_start_versor = pose_to_versor(left_start)
    left_end_versor = pose_to_versor(left_end)

    xy_plane = cga3d.e_1 ^ cga3d.e_2 ^ cga3d.e_inf ^ cga3d.e_0
    pose_plane_right = cga3d.Vector.from_motor_estimation(
        cga3d.Vector.stack([cga3d.e_0, cga3d.e_0, xy_plane, xy_plane]),
        cga3d.Vector.stack(
            [
                right_start_versor.apply(cga3d.e_0),
                right_end_versor.apply(cga3d.e_0),
                right_start_versor.apply(xy_plane),
                right_end_versor.apply(xy_plane),
            ]
        ),
    )
    pose_plane_left = cga3d.Vector.from_motor_estimation(
        cga3d.Vector.stack([cga3d.e_0, cga3d.e_0, xy_plane, xy_plane]),
        cga3d.Vector.stack(
            [
                left_start_versor.apply(cga3d.e_0),
                left_end_versor.apply(cga3d.e_0),
                left_start_versor.apply(xy_plane),
                left_end_versor.apply(xy_plane),
            ]
        ),
    )

    points = cga3d.Vector.from_euclid(
        [
            [
                left_start_versor.apply(cga3d.e_0).to_euclid(),
                left_end_versor.apply(cga3d.e_0).to_euclid(),
            ],
            [
                right_start_versor.apply(cga3d.e_0).to_euclid(),
                right_end_versor.apply(cga3d.e_0).to_euclid(),
            ],
        ]
    )

    planes = cga3d.Vector.stack([pose_plane_left.apply(xy_plane), pose_plane_right.apply(xy_plane)])
    edge = -planes[0].meet(planes[1])
    start_end = project_to_flat(sum(points[:, :2]), edge)
    poses = [
        cga3d.Vector.from_motor_estimation(
            cga3d.Vector.stack(
                [
                    cga3d.e_0,
                    -cga3d.e_1 ^ cga3d.e_2 ^ cga3d.e_0 ^ cga3d.e_inf,
                    cga3d.e_0 ^ cga3d.e_1 ^ cga3d.e_inf,
                ]
            ),
            cga3d.Vector.stack([start_end[i], planes[i], edge]),
        )
        for i in range(2)
    ]
    return versor_to_pose(poses[0]), versor_to_pose(poses[1])


@register_builtin_func()
def find_edge(  # pylint: disable=too-many-positional-arguments
    left_start: Vector3d,
    left_end: Vector3d,
    left_auxiliary: Vector3d,
    right_start: Vector3d,
    right_end: Vector3d,
    right_auxiliary: Vector3d,
) -> tuple[Pose, Pose]:
    # TODO: 6 points? What 6 points? Is there a typo?
    """Given 6 points, two poses describing the edge are returned

    Args:
        left_start: approximate start position at the left plane
        left_end: approximate end position at the left plane
        left_auxiliary: a third point on the left plane
        right_start: approximate start position at the right plane
        right_end: approximate end position at the right plane
        right_auxiliary: a third point on the right plane

    Returns:
        start_pose: a pose with position at the start and an orientation normal to the left plane
        end_pose: a pose with position at the start and an orientation normal to the right plane
    """
    points = cga3d.Vector.from_euclid(
        [[left_start, left_end, left_auxiliary], [right_start, right_end, right_auxiliary]]
    )
    planes = points[:, 0] ^ points[:, 1] ^ points[:, 2] ^ cga3d.e_inf * np.array([1, -1])
    edge = -planes[0].meet(planes[1])
    start_end = project_to_flat(sum(points[:, :2]), edge)
    poses = [
        cga3d.Vector.from_motor_estimation(
            cga3d.Vector.stack(
                [
                    cga3d.e_0,
                    -cga3d.e_1 ^ cga3d.e_2 ^ cga3d.e_0 ^ cga3d.e_inf,
                    cga3d.e_0 ^ cga3d.e_1 ^ cga3d.e_inf,
                ]
            ),
            cga3d.Vector.stack([start_end[i], planes[i], edge]),
        )
        for i in range(2)
    ]
    return versor_to_pose(poses[0]), versor_to_pose(poses[1])
