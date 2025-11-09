from nova import Nova
from nova.actions import ptp
from nova.types.motion_settings import MotionSettings
from nova.types.pose import Pose
from nova.core import logger
import asyncio
import pytest


@pytest.mark.asyncio
async def test_task_cancelation_during_move_to_start_position():
    async with Nova() as nova:
        cell = nova.cell()
        ur = await cell.controller("ur")
        async with ur[0] as mg:
            tcp = "Flange"
            # move the robot far away from the start position
            await mg.plan_and_execute(
                actions=ptp(Pose((-650, 500, 250, 0, 0, 0))),
                tcp=tcp
            )


            logger.info("first movement done")
            # start a trajectory execution which will move to the start position
            actions = [
                ptp(Pose((650, 500, 250, 0, 0, 0)), settings=MotionSettings(tcp_velocity_limit=50)),
            ]
            tcp = "Flange"
            # move the robot far away from the start position
            trajectory = await mg.plan(actions=actions, tcp=tcp)

            movement_started = asyncio.Event()
            movement_finished = asyncio.Event()
            async def robot_movement():
                try:
                    async for _  in mg.stream_execute(trajectory, actions=actions, tcp=tcp):
                        if not movement_started.is_set():
                            movement_started.set()
                except asyncio.CancelledError:
                    movement_finished.set()
                    raise


            execution_task = asyncio.create_task(robot_movement())
            movement_started.wait()
            await asyncio.sleep(2)

            task_canceled = False
            try:
                execution_task.cancel()
                await execution_task
            except asyncio.CancelledError:
                assert movement_finished.is_set(), "Movement was not cancelled properly"
                task_canceled = True
            
            assert task_canceled, "Task was not cancelled properly"
            await asyncio.sleep(10)




@pytest.mark.asyncio
async def test_task_cancelation_during_during_execute_trajectory():
    async with Nova() as nova:
        cell = nova.cell()
        ur = await cell.controller("ur")
        async with ur[0] as mg:
            actions = [
                ptp(Pose((650, 500, 250, 0, 0, 0)), settings=MotionSettings(tcp_velocity_limit=50)),
            ]
            tcp = "Flange"
            trajectory = await mg.plan(actions=actions, tcp=tcp)

            execution_task = asyncio.create_task(mg.execute(trajectory, actions=actions, tcp=tcp))
            await asyncio.sleep(2)
            try:
                result = execution_task.cancel()
                logger.info(f"Cancelation sent: {result}")
                await execution_task
            except asyncio.CancelledError as e:
                logger.info(f"Task was cancelled: {e}")


            await asyncio.sleep(10)
            #await execution_task


   