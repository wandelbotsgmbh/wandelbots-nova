import asyncio
import signal

import wandelbots_api_client.v2 as wb_v2
from decouple import config
from faststream import FastStream
from faststream.nats import NatsBroker
from icecream import ic
from wandelbots_api_client import models as wbmodels
from wandelbots_api_client.v2 import models as wbmodels_v2

import nova
from nova.actions import jnt, ptp
from nova.cell import virtual_controller
from nova.core.nova import Nova
from nova.program import ProgramPreconditions
from nova.types import MotionSettings, Pose
from nova.utils.runtime import stoppable_run

ic.configureOutput(includeContext=True)

NATS_BROKER = config("NATS_BROKER")


# Configure the robot program
@nova.program(
    name="start_here",
    # viewer=nova.viewers.Rerun(),  # add this line for a 3D visualization
    preconditions=ProgramPreconditions(
        controllers=[
            virtual_controller(
                name="kuka-kr16-r2010",
                manufacturer=wbmodels.Manufacturer.KUKA,
                type=wbmodels.VirtualControllerTypes.KUKA_MINUS_KR16_R2010_2,
            )
        ],
        cleanup_controllers=False,
    ),
)
async def start():
    """Main robot control function."""
    async with Nova() as nova:
        cell = nova.cell()
        controller = await cell.controller("kuka-kr16-r2010")

        v2_config = nova._api_client._api_client.configuration
        v2_config.host = v2_config.host[:-1] + "2"
        v2_api_client: wb_v2.ApiClient = wb_v2.ApiClient(v2_config)
        bus_io_api = wb_v2.BUSInputsOutputsApi(v2_api_client)
        ic(await bus_io_api.get_bus_io_values(cell.cell_id, ios=["b0"]))

        async with controller[0] as motion_group:
            home_joints = await motion_group.joints()
            tcp_names = await motion_group.tcp_names()
            tcp = tcp_names[0]

            # Get current TCP pose and create target poses
            current_pose = await motion_group.tcp_pose(tcp)
            target_pose = current_pose @ Pose((1, 0, 0, 0, 0, 0))

            # Define movement sequence
            actions = [
                jnt(home_joints),  # Move to home position
                ptp(target_pose),  # Move to target pose
                jnt(home_joints),  # Return to home
                ptp(target_pose @ [100, 0, 0, 0, 0, 0]),  # Move 100mm in target pose's local x-axis
                jnt(home_joints),
                ptp(target_pose @ (100, 100, 0, 0, 0, 0)),  # Move 100mm in local x and y axes
                jnt(home_joints),
                ptp(target_pose @ Pose((0, 100, 0, 0, 0, 0))),  # Move 100mm in local y-axis only
                jnt(home_joints),
            ]

            # Set motion velocity for all actions
            for action in actions:
                action.settings = MotionSettings(tcp_velocity_limit=200)

            # Plan the movements (shows in 3D viewer or creates an rrd file)
            joint_trajectory = await motion_group.plan(actions, tcp)

            # OPTIONAL: Execute the planned movements
            # You can comment out the lines below to only see the plan in Rerun
            while True:
                print("Executing planned movements...")
                await motion_group.execute(joint_trajectory, tcp, actions=actions)
                print("Movement execution completed!")


def sigint_handler(sig, frame):
    print("Received SIGINT, stopping program...")
    ic(sig, frame)
    # program_stop_evt.set()


async def main():
    signal.signal(signal.SIGINT, sigint_handler)
    broker = NatsBroker(NATS_BROKER, apply_types=False)
    faststream_app = FastStream(broker)
    faststream_app_ready_evt = asyncio.Event()
    program_stop_evt = asyncio.Event()

    @faststream_app.after_startup
    async def after_startup():
        faststream_app_ready_evt.set()

    @broker.subscriber("nova.v2.cells.cell.bus-ios.ios")
    async def collision_stopper(msg_body: list[wbmodels_v2.IOValue]):
        ic(msg_body, msg_body[0])
        value = wbmodels_v2.IOValue.from_dict(msg_body[0]).actual_instance
        if value.io == "b0" and value.value == False:
            program_stop_evt.set()

    async def exit_wrapper(coro):
        try:
            await coro
        finally:
            faststream_app.exit()

    async with asyncio.TaskGroup() as tg:
        tg.create_task(faststream_app.run(), name="faststream_app")
        await faststream_app_ready_evt.wait()
        tg.create_task(stoppable_run(exit_wrapper(start()), program_stop_evt.wait()))

    ic()


if __name__ == "__main__":
    asyncio.run(main())
