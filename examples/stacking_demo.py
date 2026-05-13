# package imports
import nova
import wandelbots_isaacsim_api as isaac_sim_api

import wandelbots_isaacsim_api.trajectory as trajectory_utils
from nova import run_program
from nova import api
from nova.actions import cartesian_ptp, io_write, linear, wait
from nova.actions.io import WriteAction
from nova.actions.mock import WaitAction
from nova.actions.motions import CartesianPTP, Linear
from nova.cell import virtual_controller
from nova.cell.cell import Cell
from nova.cell.controller import Controller
from nova.cell.motion_group import MotionGroup
from nova.types.motion_settings import MotionSettings
from nova.types.pose import Pose
from wandelbots_isaacsim_api.api.teaching_api import TeachingApi
from wandelbots_isaacsim_api.models.ghost_object import GhostObject
import logging

logger = logging.getLogger(__name__)

# parameter definitions
isaacsim_ip_address = "<put your isaac sim ip address here>"
omniservice_host = f"http://{isaacsim_ip_address}:8011/omniservice/api/v2"
controller_name = "kuka"
robot_prim_path = "/World/cell/workspace_kuka/KUKA_KR10_R900_2"
manufacturer = api.models.Manufacturer.KUKA
robot_model = "kuka-kr10_r900_2"
gripper_signal = "OUT#1"
motion_group_num = 0

# helper function to get poses from ghost objects in isaac sim
async def get_poses_from_ghost_objects(
    isaac_sim_api_url: str, robot_prim_path: str
) -> dict[str, list[Pose]]:
    """
    Helper function to get pick and place poses from ghost objects in the simulation model.
    Returns a dictionary mapping each unique ghost object name to a list of Pose objects.
    """
    async with isaac_sim_api.ApiClient(
        configuration=isaac_sim_api.Configuration(host=isaac_sim_api_url)
    ) as isaac_sim_api_client:
        teaching_api: TeachingApi = isaac_sim_api.TeachingApi(
            api_client=isaac_sim_api_client
        )
        ghost_objects: list[GhostObject] | None = None
        ghost_objects = await teaching_api.list_ghost_objects(
            relative_to_prim=robot_prim_path
        )

        # Create a dictionary mapping each unique ghost object name to a list of Pose objects
        poses_dict: dict[str, list[Pose]] = {}
        for ghost_obj in ghost_objects:
            name = ghost_obj.name
            pose_obj = Pose(tuple(ghost_obj.pose.pose))
            if name not in poses_dict:
                poses_dict[name] = []
            poses_dict[name].append(pose_obj)
        return poses_dict


# configure the robot program
@nova.program(
    id="stacking_demo",
    name="Stacking Demo",
    viewer=trajectory_utils.TrajectoryViewer(
        omniverse_host=omniservice_host,
        motion_group_prim_paths={
            f"{motion_group_num}@{controller_name}": robot_prim_path,
        },
    ),
    preconditions=nova.ProgramPreconditions(
        controllers=[
            virtual_controller(
                name=controller_name,
                manufacturer=manufacturer,
                type=robot_model,
            )
        ],
        cleanup_controllers=False,
    )
)
async def start(ctx: nova.ProgramContext) -> None:
    """Main robot control function."""

    # get cell, controller, and motion group instances from the Nova instance
    cell: Cell = ctx.cell
    controller: Controller = await cell.controller(controller_name)
    motion_group: MotionGroup = controller[motion_group_num]

    # define motion speed and tcp
    fast: MotionSettings = MotionSettings(tcp_velocity_limit=250)
    tcp = "schunk_coact_gripper"

    # get ghost objects from isaac sim
    robot_poses: dict[str, list[Pose]] = await get_poses_from_ghost_objects(
        isaac_sim_api_url=omniservice_host,
        robot_prim_path=robot_prim_path,
    )

    # iterate over ghost objects and run pick and place motion commands
    pick_poses = [pose for key, poses in robot_poses.items() if "PickPose" in key for pose in poses]
    place_poses = [pose for key, poses in robot_poses.items() if "PlacePose" in key for pose in poses]
    for (pick_pose, place_pose) in zip(pick_poses, place_poses):
        # define robot motion sequence
        actions: list[CartesianPTP | Linear | WriteAction | WaitAction] = [
            cartesian_ptp(
                target=pick_pose @ Pose(0, 0, -200, 0, 0, 0),
                settings=fast,
            ),
            linear(target=pick_pose, settings=fast),
            io_write(key=gripper_signal, value=True),
            wait(wait_for_in_seconds=2),
            linear(
                target=pick_pose @ Pose(0, 0, -200, 0, 0, 0),
                settings=fast,
            ),
            cartesian_ptp(
                target=place_pose @ Pose(0, 0, -200, 0, 0, 0),
                settings=fast,
            ),
            linear(target=place_pose, settings=fast),
            io_write(key=gripper_signal, value=False),
            wait(wait_for_in_seconds=2),
            linear(
                target=place_pose @ Pose(0, 0, -200, 0, 0, 0),
                settings=fast,
            ),

        ]
        await motion_group.plan_and_execute(actions, tcp)

# execute program
if __name__ == "__main__":
    run_program(program=start)