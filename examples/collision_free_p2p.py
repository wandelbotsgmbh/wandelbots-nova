import asyncio
from typing import cast

import numpy as np
import rerun as rr

import nova
from nova import Nova, api, run_program
from nova.actions import Action
from nova.actions.motions import Motion, cartesian_ptp, collision_free
from nova.cell import virtual_controller
from nova.types import MotionSettings, Pose


async def build_collision_world(
    nova: Nova, cell_name: str, motion_group_description: api.models.MotionGroupDescription
) -> str:
    store_collision_components_api = nova.api.store_collision_components_api
    store_collision_setups_api = nova.api.store_collision_setups_api
    motion_group_models_api = nova.api.motion_group_models_api

    motion_group_model = motion_group_description.motion_group_model.root

    # define annoying obstacle
    sphere_collider = api.models.Collider(
        shape=api.models.Sphere(radius=100, shape_type="sphere"),
        pose=api.models.Pose(
            position=api.models.Vector3d([-100, -500, 200]),
            orientation=api.models.RotationVector([0, 0, 0]),
        ),
    )
    await store_collision_components_api.store_collider(
        cell=cell_name, collider="annoying_obstacle", collider2=sphere_collider
    )

    # define TCP collider geometry
    tool_collider = api.models.Collider(
        shape=api.models.Box(
            size_x=100, size_y=100, size_z=100, shape_type="box", box_type=api.models.BoxType.FULL
        )
    )
    await store_collision_components_api.store_collision_tool(
        cell=cell_name, tool="tool_box", request_body={"tool_collider": tool_collider}
    )

    # define robot link geometries
    robot_link_colliders = await motion_group_models_api.get_motion_group_collision_model(
        motion_group_model=motion_group_model
    )
    await store_collision_components_api.store_collision_link_chain(
        cell=cell_name, link_chain="robot_links", collider=robot_link_colliders
    )

    # assemble scene
    collision_setup = api.models.CollisionSetup(
        colliders=api.models.ColliderDictionary({"annoying_obstacle": sphere_collider}),
        tool=api.models.Tool({"tool_geometry": tool_collider}),
        link_chain=api.models.LinkChain(
            list(api.models.Link(link) for link in robot_link_colliders)
        ),
    )
    scene_id = "collision_scene"
    await store_collision_setups_api.store_collision_setup(
        cell=cell_name, setup="collision_scene", collision_setup=collision_setup
    )
    return scene_id


@nova.program(
    viewer=nova.viewers.Rerun(),
    preconditions=nova.ProgramPreconditions(
        controllers=[
            virtual_controller(
                name="ur5",
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type=api.models.VirtualControllerTypes.UNIVERSALROBOTS_UR5E,
            )
        ],
        cleanup_controllers=False,
    ),
)
async def collision_free_p2p(ctx: nova.ProgramContext) -> None:
    """
    Example of planning a collision free PTP motion. A sphere is placed in the robot's path and the robot uses collision free p2p to move around it.
    """
    nova = ctx.nova
    cell = nova.cell()
    controller = await cell.controller("ur5")

    await ctx.nova.api.virtual_robot_setup_api.set_virtual_controller_mounting(
        cell=cell.id,
        controller=controller.id,
        motion_group=f"0@{controller.id}",
        coordinate_system=api.models.CoordinateSystem(
            name="mounting",
            coordinate_system="world",
            position=api.models.Vector3d([0, 0, 0]),
            orientation=api.models.Orientation([0, 0, 0]),
            orientation_type=api.models.OrientationType.EULER_ANGLES_EXTRINSIC_XYZ,
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
        store_collision_setups_api = ctx.nova.api.store_collision_setups_api
        collision_setup = await store_collision_setups_api.get_stored_collision_setup(
            cell="cell", setup=collision_scene_id
        )
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

        rr.log("motion/target_", rr.Points3D([[-500, -400, 200]], radii=[10], colors=[(0, 255, 0)]))

        # Use default planner to move to the left of the sphere
        # -> this will collide
        # only plan don't move
        collision_actions: list[Action] = [
            cartesian_ptp(
                target=Pose((-500, -400, 200, np.pi, 0, 0)), collision_setup=collision_setup
            )
        ]

        for action in collision_actions:
            cast(Motion, action).settings = MotionSettings(tcp_velocity_limit=200)

        try:
            await motion_group.plan(
                collision_actions,
                tcp,
                start_joint_position=joint_trajectory.joint_positions[-1].root,
            )
        except Exception as e:
            print(f"Planning failed, we continue with the collision avoidance: {e}")

        # Plan collision free PTP motion around the sphere
        welding_actions: list[Action] = [
            collision_free(
                target=Pose(-500, -400, 200, np.pi, 0, 0),
                collision_setup=collision_setup,
                settings=MotionSettings(tcp_velocity_limit=30),
            )
        ]

        await motion_group.plan(
            welding_actions, tcp=tcp, start_joint_position=joint_trajectory.joint_positions[-1].root
        )


if __name__ == "__main__":
    run_program(collision_free_p2p)
