import asyncio
from functools import partial
from pathlib import Path

import nova
import wandelbots_api_client.models as v1models
from pydantic import TypeAdapter

from nova import run_program
from nova.actions import MovementControllerContext
from nova.cell import virtual_controller
from nova.core.exceptions import InitMovementFailed
from nova.program import ProgramPreconditions
from nova.types import (
    ExecuteTrajectoryRequestStream,
    ExecuteTrajectoryResponseStream,
    MovementControllerFunction,
)

ROBOT_TCP_ID = "1"
POSITIONER_TCP_ID = "0"

ROBOT_MOTION_GROUP_ID = "0@kuka"
POSITIONER_MOTION_GROUP_ID = "1@kuka"

SYNC_IO_ID = "OUT#1"


def move_forward_in_sync(
    context: MovementControllerContext, start_on_io: v1models.StartOnIO
) -> MovementControllerFunction:
    """
    This is a copy of the default movement controller with added start_on_io handling.
    """

    async def movement_controller(
        response_stream: ExecuteTrajectoryResponseStream,
    ) -> ExecuteTrajectoryRequestStream:
        # The first request is to initialize the movement
        yield v1models.InitializeMovementRequest(trajectory=context.motion_id, initial_location=0)

        # then we get the response
        initialize_movement_response = await anext(response_stream)
        if isinstance(
            initialize_movement_response.actual_instance, v1models.InitializeMovementResponse
        ):
            r1 = initialize_movement_response.actual_instance
            if not r1.init_response.succeeded:
                raise InitMovementFailed(r1.init_response)

        # The second request is to start the movement
        set_io_list = context.combined_actions.to_set_io()
        yield v1models.StartMovementRequest(
            set_ios=set_io_list, start_on_io=start_on_io, pause_on_io=None
        )

        # then we wait until the movement is finished
        async for execute_trajectory_response in response_stream:
            instance = execute_trajectory_response.actual_instance
            # Stop when standstill indicates motion ended
            if isinstance(instance, v1models.Standstill):
                if instance.standstill.reason == v1models.StandstillReason.REASON_MOTION_ENDED:
                    return

    return movement_controller


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
        # Give some time to ensure both controllers are ready and waiting for the IO signal
        # Ideally, feedback from movement controllers would be used to ensure readiness
        await asyncio.sleep(1)
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

        # Creating movement controllers with the start_on_io condition
        start_on_io = v1models.StartOnIO(
            io=v1models.IOValue(io=SYNC_IO_ID, boolean_value=True),
            comparator=v1models.Comparator.COMPARATOR_EQUALS,
        )
        robot_controller = partial(move_forward_in_sync, start_on_io=start_on_io)
        positioner_controller = partial(move_forward_in_sync, start_on_io=start_on_io)

        # Starting both movements concurrently
        robot_trajectory_exec = asyncio.create_task(
            robot.execute(robot_path, ROBOT_TCP_ID, [], robot_controller)
        )
        positioner_trajectory_exec = asyncio.create_task(
            positioner.execute(positioner_path, POSITIONER_TCP_ID, [], positioner_controller)
        )

        # Triggering the IO signal to start both movements
        await set_io()
        await asyncio.gather(robot_trajectory_exec, positioner_trajectory_exec)


if __name__ == "__main__":
    run_program(multi_motion_group_trajectory)
