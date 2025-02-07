import asyncio

import numpy as np
import rerun as rr
from nova import MotionSettings
from nova.actions import ptp
from nova.api import models
from nova.core.exceptions import PlanTrajectoryFailed
from nova.core.nova import Nova
from nova.types import Pose
from wandelbots_api_client.models import PlanCollisionFreePTPRequest

from nova_rerun_bridge import NovaRerunBridge

"""
Example of planning a collision free PTP motion. A sphere is placed in the robot's path and the robot uses collision free p2p to move around it.
"""


async def build_collision_world(
    nova: Nova, cell_name: str, robot_setup: models.OptimizerSetup
) -> str:
    collision_api = nova._api_client.store_collision_components_api
    scene_api = nova._api_client.store_collision_scenes_api

    # define annoying obstacle
    sphere_collider = models.Collider(
        shape=models.ColliderShape(models.Sphere2(radius=100, shape_type="sphere")),
        pose=models.Pose2(position=[-100, -500, 200]),
    )
    await collision_api.store_collider(
        cell=cell_name, collider="annoying_obstacle", collider2=sphere_collider
    )

    # define TCP collider geometry
    tool_collider = models.Collider(
        shape=models.ColliderShape(
            models.Box2(size_x=100, size_y=100, size_z=100, shape_type="box", box_type="FULL")
        )
    )
    await collision_api.store_collision_tool(
        cell=cell_name, tool="tool_box", request_body={"tool_collider": tool_collider}
    )

    # define robot link geometries
    robot_link_colliders = await collision_api.get_default_link_chain(
        cell=cell_name, motion_group_model=robot_setup.motion_group_type
    )
    await collision_api.store_collision_link_chain(
        cell=cell_name, link_chain="robot_links", collider=robot_link_colliders
    )

    # assemble scene
    scene = models.CollisionScene(
        colliders={"annoying_obstacle": sphere_collider},
        motion_groups={
            "motion_group": models.CollisionMotionGroup(
                tool={"tool_geometry": tool_collider}, link_chain=robot_link_colliders
            )
        },
    )
    scene_id = "collision_scene"
    await scene_api.store_collision_scene(
        cell_name, scene_id, models.CollisionSceneAssembly(scene=scene)
    )
    return scene_id


async def test():
    async with Nova() as nova, NovaRerunBridge(nova) as bridge:
        await bridge.setup_blueprint()

        cell = nova.cell()
        controller = await cell.ensure_virtual_robot_controller(
            "ur5",
            models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR5E,
            models.Manufacturer.UNIVERSALROBOTS,
        )
        # Connect to the controller and activate motion groups
        async with controller[0] as motion_group:
            tcp = "Flange"

            robot_setup: models.OptimizerSetup = await motion_group._get_optimizer_setup(tcp=tcp)
            robot_setup.safety_setup.global_limits.tcp_velocity_limit = 200

            collision_scene_id = await build_collision_world(nova, "cell", robot_setup)

            await bridge.log_collision_scenes()

            # Use default planner to move to the right of the sphere
            home_joints = await motion_group.joints()
            home = await motion_group.tcp_pose(tcp)
            actions = [ptp(home), ptp(target=Pose((300, -400, 200, np.pi, 0, 0)))]

            for action in actions:
                action.settings = MotionSettings(tcp_velocity_limit=200)

            try:
                joint_trajectory = await motion_group.plan(actions, tcp)
                await bridge.log_actions(actions)
                await bridge.log_trajectory(joint_trajectory, tcp, motion_group)
                await motion_group.execute(joint_trajectory, tcp, actions=actions)
            except PlanTrajectoryFailed as e:
                await bridge.log_actions(actions)
                await bridge.log_trajectory(e.error.joint_trajectory, tcp, motion_group)
                await bridge.log_error_feedback(e.error.error_feedback)

            rr.log(
                "motion/target_", rr.Points3D([[-500, -400, 200]], radii=[10], colors=[(0, 255, 0)])
            )

            # Use default planner to move to the left of the sphere
            # -> this will collide
            # only plan don't move
            actions = [ptp(home), ptp(target=Pose((-500, -400, 200, np.pi, 0, 0)))]

            for action in actions:
                action.settings = MotionSettings(tcp_velocity_limit=200)

            try:
                joint_trajectory = await motion_group.plan(actions, tcp)
                await bridge.log_actions(actions)
                await bridge.log_trajectory(joint_trajectory, tcp, motion_group)
            except PlanTrajectoryFailed as e:
                await bridge.log_actions(actions)
                await bridge.log_trajectory(e.error.joint_trajectory, tcp, motion_group)
                await bridge.log_error_feedback(e.error.error_feedback)

            # Plan collision free PTP motion around the sphere
            scene_api = nova._api_client.store_collision_scenes_api
            collision_scene = await scene_api.get_stored_collision_scene(
                cell="cell", scene=collision_scene_id
            )

            planTrajectory: models.PlanTrajectoryResponse = (
                await nova._api_client.motion_api.plan_collision_free_ptp(
                    cell="cell",
                    plan_collision_free_ptp_request=PlanCollisionFreePTPRequest(
                        robot_setup=robot_setup,
                        start_joint_position=home_joints,
                        target=models.PlanCollisionFreePTPRequestTarget(
                            models.Pose2(position=[-500, -400, 200], orientation=[np.pi, 0, 0])
                        ),
                        static_colliders=collision_scene.colliders,
                        collision_motion_group=collision_scene.motion_groups["motion_group"],
                    ),
                )
            )

            if isinstance(
                planTrajectory.response.actual_instance, models.PlanTrajectoryFailedResponse
            ):
                joint_trajectory = planTrajectory.response.actual_instance.joint_trajectory
                await bridge.log_trajectory(joint_trajectory, tcp, motion_group)
                raise PlanTrajectoryFailed(planTrajectory.response.actual_instance)

            joint_trajectory = planTrajectory.response.actual_instance
            await bridge.log_trajectory(joint_trajectory, tcp, motion_group)


if __name__ == "__main__":
    asyncio.run(test())
