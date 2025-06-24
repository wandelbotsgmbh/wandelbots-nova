import asyncio

from nova_python_app.programs.common_code import open_laser

from nova import Nova
from nova.actions import cartesian_ptp, joint_ptp
from nova.api import models
from nova.cell import virtual_controller
from nova.types import MotionSettings, Pose
from novax import BaseProgramModel, parse_model_from_args, program


class ProgramModel(BaseProgramModel):
    start_pose: list[float]
    end_pose: list[float]


@program(name="robot_movement_template_2", model=ProgramModel)
async def main(model: ProgramModel):
    async with Nova(host="http://172.31.12.193") as nova:
        open_laser()
        cell = nova.cell()
        controller = await cell.ensure_controller(
            robot_controller=virtual_controller(
                name="ur",
                manufacturer=models.Manufacturer.UNIVERSALROBOTS,
                type=models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
            )
        )

        # Connect to the controller and activate motion groups
        async with controller[0] as motion_group:
            home_joints = await motion_group.joints()
            tcp_names = await motion_group.tcp_names()
            tcp = tcp_names[0]

            # Get current TCP pose and offset it slightly along the x-axis
            current_pose = await motion_group.tcp_pose(tcp)
            target_pose = current_pose @ Pose((1, 0, 0, 0, 0, 0))

            actions = [
                joint_ptp(home_joints),
                cartesian_ptp(target_pose),
                joint_ptp(home_joints),
                cartesian_ptp(target_pose @ [50, 0, 0, 0, 0, 0]),
                joint_ptp(home_joints),
                cartesian_ptp(target_pose @ (50, 100, 0, 0, 0, 0)),
                joint_ptp(home_joints),
                cartesian_ptp(target_pose @ Pose((0, 50, 0, 0, 0, 0))),
                joint_ptp(home_joints),
            ]

        # you can update the settings of the action
        for action in actions:
            action.settings = MotionSettings(tcp_velocity_limit=200)

        joint_trajectory = await motion_group.plan(actions, tcp)
        motion_iter = motion_group.stream_execute(joint_trajectory, tcp, actions=actions)
        async for motion_state in motion_iter:
            print(motion_state)


if __name__ == "__main__":
    model = parse_model_from_args(ProgramModel)
    asyncio.run(main(model))
