import asyncio
from pathlib import Path

import wandelbots_api_client.models as v1models
from pydantic import TypeAdapter

import nova
from nova import run_program
from nova.cell import virtual_controller
from nova.program import ProgramPreconditions

ROBOT_TCP_ID = "1"
POSITIONER_TCP_ID = "0"

ROBOT_MOTION_GROUP_ID = "0@kuka"
POSITIONER_MOTION_GROUP_ID = "1@kuka"

SYNC_IO_ID = "OUT#1"


def load_controller_config() -> str:
    path = Path(__file__).parent / "multi_motion_group_controller.json"
    return path.read_text()


def load_joint_trajectories() -> dict[str, v1models.JointTrajectory]:
    path = Path(__file__).parent / "multi_motion_group_trajectory.json"
    adapter = TypeAdapter(dict[str, v1models.JointTrajectory])
    return adapter.validate_json(path.read_text())


@nova.program(
    name="multi_motion_group",
    preconditions=ProgramPreconditions(
        controllers=[
            virtual_controller(
                name="kuka",
                manufacturer=nova.api.models.Manufacturer.KUKA,
                type=nova.api.models.VirtualControllerTypes.KUKA_MINUS_KR210_R2700_2,
                json=load_controller_config(),
            )
        ]
    ),
)
async def multi_motion_group_trajectory():
    """
    Example of synchronized trajectory execution with two motion groups (robot and positioner).
    """

    async def set_io():
        print("Setting sync IO to True")
        await controller.write(key=SYNC_IO_ID, value=True)

    async with nova.Nova() as n:
        cell = n.cell()
        controller = await cell.controller("kuka")
        robot = controller.motion_group(ROBOT_MOTION_GROUP_ID)
        positioner = controller.motion_group(POSITIONER_MOTION_GROUP_ID)

        # Load trajectories for both motion groups (same duration expected).
        trajectories = load_joint_trajectories()
        try:
            robot_path = trajectories["manipulator_path"]
            positioner_path = trajectories["positioner_path"]
        except KeyError as missing_name:
            raise KeyError(
                f"Trajectory '{missing_name.args[0]}' not found in multi_motion_group_trajectory.json"
            ) from missing_name

        # Resetting the sync IO to False before starting
        await controller.write(key=SYNC_IO_ID, value=False)

        print("Starting synchronized execution...")

        # Starting both movements concurrently with waiting for IO
        start_on_io = v1models.StartOnIO(
            io=v1models.IOValue(io=SYNC_IO_ID, boolean_value=True),
            comparator=v1models.Comparator.COMPARATOR_EQUALS,
        )
        robot_trajectory_exec = asyncio.create_task(
            robot.execute(robot_path, ROBOT_TCP_ID, [], start_on_io=start_on_io)
        )
        positioner_trajectory_exec = asyncio.create_task(
            positioner.execute(positioner_path, POSITIONER_TCP_ID, [], start_on_io=start_on_io)
        )

        # Give some time to ensure both controllers are ready and waiting for the IO signal
        # Ideally, feedback from movement controllers would be used to ensure readiness
        await asyncio.sleep(1)

        # Triggering the IO signal to start both movements
        await set_io()
        await asyncio.gather(robot_trajectory_exec, positioner_trajectory_exec)


if __name__ == "__main__":
    run_program(multi_motion_group_trajectory)
