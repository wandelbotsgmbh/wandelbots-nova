import asyncio
from datetime import datetime

from icecream import ic
from wandelbots_api_client.v2_pydantic.models.models import SettableRobotSystemMode

import nova
from nova import run_program
from nova.actions.motions import cartesian_ptp, joint_ptp
from nova.cell.controllers import kuka_controller, virtual_controller
from nova.config import CELL_NAME
from nova.program import ProgramPreconditions
from nova.types.motion_settings import MotionSettings
from nova.types.pose import Pose
from nova.types.vector3d import Vector3d

ic.configureOutput(includeContext=True, prefix=lambda: f"{datetime.now()} | ")


CONTROLLER_NAME = "kuka"
# Default motion parameters used when controllers are discovered dynamically.
DEFAULT_TCP = "1"
DEFAULT_Z_OFFSET_MM = 1.0
DEFAULT_VELOCITY = 50.0
DEFAULT_ACCELERATION = 250.0
DEFAULT_TCP_JERK = 2500.0
DEFAULT_WAIT_IO = "start"
DEFAULT_WAIT_IO_VALUE = True
DEFAULT_STOP_IO = "stop"
DEFAULT_STOP_IO_VALUE = True

# Default linear-axis motion parameters.
DEFAULT_AXIS_TRAVEL_MM = 1.0
DEFAULT_AXIS_VELOCITY = 200.0  # mm/s for the prismatic joint


phys_controller = kuka_controller(
    name=CONTROLLER_NAME,
    controller_ip="192.168.101.131",
    controller_port=54600,
    rsi_server_ip="192.168.102.130",
    rsi_server_port=30152,
)

virt_controller = virtual_controller(
    name=CONTROLLER_NAME, manufacturer=nova.api.models.Manufacturer.KUKA, type="kuka-kr240-r2900"
)


@nova.program(
    name="Minimal Program",
    preconditions=ProgramPreconditions(controllers=[virt_controller], cleanup_controllers=False),
)
async def main(ctx: nova.ProgramContext):
    cell = ctx.nova.cell()
    controller = await cell.controller(CONTROLLER_NAME)

    mg_robot = controller[0]
    mg_axis = controller[1]

    # Switch controller default usage to control mode before signal waiting begins.
    await ctx.nova.api.controller_api.set_default_mode(
        cell=CELL_NAME, controller=CONTROLLER_NAME, mode=SettableRobotSystemMode.MODE_CONTROL
    )

    robot_settings = MotionSettings(
        tcp_velocity_limit=DEFAULT_VELOCITY,
        tcp_acceleration_limit=DEFAULT_ACCELERATION,
        tcp_jerk_limit=DEFAULT_TCP_JERK,
    )
    axis_settings = MotionSettings(
        joint_velocity_limits=(DEFAULT_AXIS_VELOCITY,),
        joint_acceleration_limits=(DEFAULT_ACCELERATION,),
        joint_jerk_limits=(DEFAULT_TCP_JERK,),
    )

    # Read axis limits once to determine travel direction.
    axis_desc = await mg_axis.get_description()
    auto_limits = axis_desc.operation_limits.auto_limits
    if (
        auto_limits is None
        or auto_limits.joints is None
        or auto_limits.joints[0].position is None
        or auto_limits.joints[0].position.lower_limit is None
        or auto_limits.joints[0].position.upper_limit is None
    ):
        raise ValueError(f"Linear axis 1@{CONTROLLER_NAME} has no position limits configured")
    axis_lower = auto_limits.joints[0].position.lower_limit
    axis_upper = auto_limits.joints[0].position.upper_limit

    home_pose = await mg_robot.tcp_pose(DEFAULT_TCP)
    axis_home = (await mg_axis.joints())[0]
    up_pose = Pose(
        position=Vector3d(
            x=home_pose.position.x,
            y=home_pose.position.y,
            z=home_pose.position.z + DEFAULT_Z_OFFSET_MM,
        ),
        orientation=home_pose.orientation,
    )
    axis_target = _compute_axis_target(axis_home, DEFAULT_AXIS_TRAVEL_MM, axis_lower, axis_upper)

    robot_actions = [
        cartesian_ptp(up_pose, settings=robot_settings),
        cartesian_ptp(home_pose, settings=robot_settings),
    ]
    axis_actions = [
        joint_ptp((axis_target,), settings=axis_settings),
        joint_ptp((axis_home,), settings=axis_settings),
    ]
    robot_trajectory, axis_trajectory = await asyncio.gather(
        mg_robot.plan(robot_actions, DEFAULT_TCP), mg_axis.plan(axis_actions, "0")
    )
    ic()

    for i in range(100):
        async with asyncio.TaskGroup() as tg:
            tg.create_task(mg_robot.execute(robot_trajectory, DEFAULT_TCP, robot_actions))
            tg.create_task(mg_axis.execute(axis_trajectory, "0", axis_actions))
    ic()


def _compute_axis_target(
    current_pos: float, travel_mm: float, lower_limit: float, upper_limit: float
) -> float:
    """Compute the axis target position, moving toward the farther end.

    The direction is chosen so the axis moves toward whichever limit end
    is farther from the current position.  The travel distance is clamped
    to stay within the axis range.
    """
    dist_to_lower = abs(current_pos - lower_limit)
    dist_to_upper = abs(upper_limit - current_pos)

    if dist_to_upper >= dist_to_lower:
        # Move toward the upper limit.
        return min(current_pos + travel_mm, upper_limit)
    # Move toward the lower limit.
    return max(current_pos - travel_mm, lower_limit)


if __name__ == "__main__":
    run_program(main)
