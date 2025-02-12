from asyncio import run

import numpy as np
import wandelbots_api_client as nova
from decouple import config
from loguru import logger

NOVA_API_HOST = config("NOVA_API", default=None)
NOVA_ACCESS_TOKEN = config("NOVA_ACCESS_TOKEN", default=None)

# get via /api/v1/cells/{cell}
CELL_CONFIG = {
    "controllers": [
        {
            "configuration": {
                "kind": "VirtualController",
                "manufacturer": "universalrobots",
                "position": "[1.17,-1.658,1.405,-1.571,-1.571,1.17,0]",
                "type": "universalrobots-ur5e",
            },
            "name": "ur",
        }
    ],
    "name": "cell",
}


def get_api_client(nova_host: str) -> nova.ApiClient:
    api_base_path = "/api/v1"
    base_url = f"{nova_host}{api_base_path}"
    client_config = nova.Configuration(host=base_url, access_token=NOVA_ACCESS_TOKEN)
    client_config.verify_ssl = False
    return nova.ApiClient(client_config)


async def get_motion_group_infos(
    client: nova.ApiClient, cell_name: str, tcp_id: str
) -> tuple[str, nova.models.OptimizerSetup]:
    controller_api = nova.ControllerApi(client)
    motion_group_api = nova.MotionGroupApi(client)
    motion_group_info_api = nova.MotionGroupInfosApi(client)

    controllers = (await controller_api.list_controllers(cell=cell_name)).instances
    motion_group_instances = await motion_group_api.activate_all_motion_groups(
        cell=cell_name, controller=controllers[0].controller
    )
    motion_group_id = motion_group_instances.instances[0].motion_group
    robot_setup = await motion_group_info_api.get_optimizer_configuration(
        cell=cell_name, motion_group=motion_group_id, tcp=tcp_id
    )

    return motion_group_id, robot_setup


async def build_collision_world(
    client: nova.ApiClient, cell_name: str, robot_setup: nova.models.OptimizerSetup
) -> str:
    collision_api = nova.StoreCollisionComponentsApi(client)
    scene_api = nova.StoreCollisionScenesApi(client)

    # define static colliders, e.g. workpiece
    random_vertices = [1000, 1000, 0] + 1000 * np.random.random((1000, 3))
    collider = nova.models.Collider(
        shape=nova.models.ColliderShape(
            nova.models.ConvexHull2(vertices=random_vertices.tolist(), shape_type="convex_hull")
        )
    )
    await collision_api.store_collider(cell=cell_name, collider="workpiece", collider2=collider)

    # define TCP collider geometry
    tool_collider = nova.models.Collider(
        shape=nova.models.ColliderShape(
            nova.models.Box2(size_x=100, size_y=100, size_z=100, shape_type="box", box_type="FULL")
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
    scene = nova.models.CollisionScene(
        colliders={"workpiece": collider},
        motion_groups={
            robot_setup.motion_group_type: nova.models.CollisionMotionGroup(
                tool={"tool_geometry": tool_collider}, link_chain=robot_link_colliders
            )
        },
    )
    scene_id = "collision_scene"
    await scene_api.store_collision_scene(
        cell_name, scene_id, nova.models.CollisionSceneAssembly(scene=scene)
    )
    return scene_id


async def plan_path(
    client: nova.ApiClient, cell_name: str, scene_id: str, robot_setup: nova.models.OptimizerSetup
) -> nova.models.PlanTrajectoryResponse:
    motion_api = nova.MotionApi(client)
    scene_api = nova.StoreCollisionScenesApi(client)
    start_joints = [1.17, -1.658, 1.405, -1.571, -1.571, 1.17]
    goal_pose = [-58.4, -479.4, 500, 2.1397, 2.1397, -0.3571]
    scene = await scene_api.get_stored_collision_scene(cell_name, scene_id)

    plan_request = nova.models.PlanTrajectoryRequest(
        robot_setup=robot_setup,
        start_joint_position=start_joints,
        motion_commands=[
            nova.models.MotionCommand(
                path=nova.models.MotionCommandPath(
                    nova.models.path_line.PathLine(
                        target_pose=nova.models.Pose2(
                            position=goal_pose[:3], orientation=goal_pose[3:]
                        ),
                        path_definition_name="PathLine",
                    )
                ),
                blending=nova.models.MotionCommandBlending(
                    nova.models.motion_command_blending.BlendingPosition(
                        position_zone_radius=20, blending_name="BlendingPosition"
                    )
                ),
            )
        ],
        static_colliders=scene.colliders,
        collision_motion_group=scene.motion_groups.get(robot_setup.motion_group_type),
    )

    plan_result = await motion_api.plan_trajectory(
        cell=cell_name, plan_trajectory_request=plan_request
    )
    if isinstance(plan_result.response.actual_instance, nova.models.JointTrajectory):
        logger.info("Plan request successful.")
    else:
        error = plan_result.response.actual_instance.error_feedback.actual_instance
        logger.error(error.error_feedback_name)
        logger.error(error)

    return plan_result


async def upload_planned_motion(
    client: nova.ApiClient,
    plan_response: nova.models.PlanTrajectoryResponse,
    cell_name: str,
    motion_group_id: str,
    tcp_id: str,
) -> str:
    motion_api = nova.MotionApi(api_client=client)
    trajectory = plan_response.response.actual_instance
    planned_motion = nova.models.PlannedMotion(
        motion_group=motion_group_id,
        tcp=tcp_id,
        times=trajectory.times,
        joint_positions=[nova.models.Joints(joints=j.joints) for j in trajectory.joint_positions],
    )
    plan_response = await motion_api.load_planned_motion(
        cell=cell_name, planned_motion=planned_motion
    )

    if plan_response.plan_successful_response is not None:
        motion_id = plan_response.plan_successful_response.motion
        logger.info("Planning was successful. Motion can be executed.")
    else:
        motion_id = plan_response.plan_failed_on_trajectory_response.motion
        logger.warning("Planning failed. Motion can partly be executed.")
        logger.warning(plan_response.plan_failed_on_trajectory_response.description)

    return motion_id


async def execute_motion(client: nova.ApiClient, cell_name: str, motion_id: str):
    motion_api = nova.MotionApi(api_client=client)

    # Move to first point on trajectory
    movement_stream = motion_api.stream_move_to_trajectory_via_joint_ptp(
        cell=cell_name, motion=motion_id, location_on_trajectory=0
    )
    async for m in movement_stream:
        if m.move_response:
            continue
        logger.info(f"{m.stop_response.location_on_trajectory=}")

    # Move along trajectory
    movement_stream = motion_api.stream_move_forward(
        cell=cell_name, motion=motion_id, playback_speed_in_percent=100
    )
    async for m in movement_stream:
        if m.move_response:
            continue
        logger.info(f"{m.stop_response.location_on_trajectory=}")


async def main():
    tcp_id = "Flange"
    cell_name = CELL_CONFIG["name"]

    client = get_api_client(NOVA_API_HOST)
    motion_group_id, robot_setup = await get_motion_group_infos(client, cell_name, tcp_id)
    scene_id = await build_collision_world(client, cell_name, robot_setup)
    plan_result = await plan_path(client, cell_name, scene_id, robot_setup)
    motion_id = await upload_planned_motion(client, plan_result, cell_name, motion_group_id, tcp_id)
    await execute_motion(client, cell_name, motion_id)

    await client.close()


if __name__ == "__main__":
    run(main())
