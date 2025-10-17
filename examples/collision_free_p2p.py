import asyncio
from typing import cast

import numpy as np
import rerun as rr
from wandelbots_api_client.models import (
    CoordinateSystem,
    RotationAngles,
    RotationAngleTypes,
    Vector3d,
)

import nova
from nova import Nova, api, run_program
from nova.actions import Action
from nova.actions.motions import Motion, cartesian_ptp, collision_free
from nova.cell import virtual_controller
from nova.program import ProgramPreconditions
from nova.types import MotionSettings, Pose


async def build_collision_world(
    nova: Nova, cell_name: str, motion_group_description: api.models.MotionGroupDescription
) -> str:
    collision_api = nova._api_client.store_collision_components_api
    scene_api = nova._api_client.store_collision_setups_api

    # define annoying obstacle
    sphere_collider = api.models.Collider(
        shape=api.models.ColliderShape(api.models.Sphere2(radius=100, shape_type="sphere")),
        pose=api.models.Pose2(position=[-100, -500, 200]),
    )
    await collision_api.store_collider(
        cell=cell_name, collider="annoying_obstacle", collider2=sphere_collider
    )

    # define TCP collider geometry
    tool_collider = api.models.Collider(
        shape=api.models.ColliderShape(
            api.models.Box2(size_x=100, size_y=100, size_z=100, shape_type="box", box_type="FULL")
        )
    )
    await collision_api.store_collision_tool(
        cell=cell_name, tool="tool_box", request_body={"tool_collider": tool_collider}
    )

    # define robot link geometries
    robot_link_colliders = await collision_api.get_default_link_chain(
        cell=cell_name, motion_group_model=motion_group_description.motion_group_model
    )
    await collision_api.store_collision_link_chain(
        cell=cell_name, link_chain="robot_links", collider=robot_link_colliders
    )

    # assemble scene
    scene = api.models.CollisionScene(
        colliders={"annoying_obstacle": sphere_collider},
        motion_groups={
            motion_group_description.motion_group_model: api.models.CollisionMotionGroup(
                tool={"tool_geometry": tool_collider}, link_chain=robot_link_colliders
            )
        },
    )
    scene_id = "collision_scene"
    await scene_api.store_collision_scene(
        cell_name, scene_id, api.models.CollisionSceneAssembly(scene=scene)
    )
    return scene_id


@nova.program(
    name="collision_free_p2p",
    viewer=nova.viewers.Rerun(),
    preconditions=ProgramPreconditions(
        controllers=[
            virtual_controller(
                name="ur5",
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type=api.models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR5E,
            )
        ],
        cleanup_controllers=False,
    ),
)
async def collision_free_p2p() -> None:
    """
    Example of planning a collision free PTP motion. A sphere is placed in the robot's path and the robot uses collision free p2p to move around it.
    """
    async with Nova() as nova:
        cell = nova.cell()
        controller = await cell.controller("ur5")

        await nova._api_client.virtual_robot_setup_api.set_virtual_robot_mounting(
            cell="cell",
            controller=controller.controller_id,
            id=0,
            coordinate_system=CoordinateSystem(
                coordinate_system="world",
                name="mounting",
                reference_uid="",
                position=Vector3d(x=0, y=0, z=0),
                rotation=RotationAngles(
                    angles=[0, 0, 0], type=RotationAngleTypes.EULER_ANGLES_EXTRINSIC_XYZ
                ),
            ),
        )

        # NC-1047
        await asyncio.sleep(5)

        # Connect to the controller and activate motion groups
        async with controller[0] as motion_group:
            tcp = "Flange"

            motion_group_description: api.models.MotionGroupDescription = (
                await motion_group.get_description()
            )
            collision_scene_id = await build_collision_world(nova, "cell", motion_group_description)

            # Use default planner to move to the right of the sphere
            home = await motion_group.tcp_pose(tcp)
            actions: list[Action] = [
                cartesian_ptp(home),
                cartesian_ptp(target=Pose((300, -400, 200, np.pi, 0, 0))),
            ]

            for action in actions:
                cast(Motion, action).settings = MotionSettings(tcp_velocity_limit=200)

            joint_trajectory = await motion_group.plan(
                actions, tcp, start_joint_position=(0, -np.pi / 2, np.pi / 2, 0, 0, 0)
            )

            rr.log(
                "motion/target_", rr.Points3D([[-500, -400, 200]], radii=[10], colors=[(0, 255, 0)])
            )

            # Use default planner to move to the left of the sphere
            # -> this will collide
            # only plan don't move
            collision_actions: list[Action] = [
                cartesian_ptp(target=Pose((-500, -400, 200, np.pi, 0, 0)))
            ]

            for action in collision_actions:
                cast(Motion, action).settings = MotionSettings(tcp_velocity_limit=200)

            await motion_group.plan(
                collision_actions,
                tcp,
                start_joint_position=joint_trajectory.joint_positions[-1].joints,
            )

            # Plan collision free PTP motion around the sphere
            scene_api = nova._api_client.store_collision_setups_api
            collision_scene = await scene_api.get_stored_collision_scene(
                cell="cell", scene=collision_scene_id
            )

            welding_actions: list[Action] = [
                collision_free(
                    target=Pose((-500, -400, 200, np.pi, 0, 0)),
                    collision_scene=collision_scene,
                    settings=MotionSettings(tcp_velocity_limit=30),
                )
            ]

            await motion_group.plan(
                welding_actions,
                tcp=tcp,
                start_joint_position=joint_trajectory.joint_positions[-1].joints,
            )


if __name__ == "__main__":
    run_program(collision_free_p2p)
