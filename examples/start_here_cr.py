"""
This example runs the `inspect-30deg` routine of abb-irb-1300-demo.json on a virtual ABB IRB 1300,
once as an action list and once as the dataset's CommandRoutine, to compare both.

This demonstrates:
- Setting up a virtual robot controller
- Connecting to the robot
- Planning and executing basic movements
- Using joint and point-to-point motion types

Key robotics concepts:
- Motion groups: Controllable robot parts (usually the arm)
- TCP (Tool Center Point): The point you control on the robot
- Joint movement (jnt): Move by specifying joint angles in radians
- Point-to-point movement (ptp): Move to a specific position/orientation (x,y,z, rotation angle in radians)
- Pose: Position (x,y,z) and orientation (rx,ry,rz) in 3D space
"""

import asyncio

from nova.cell.routine_execution import resolve_routine
from pydantic import Field

import nova
from nova import api, run_program
from nova import datasets as ds
from nova.actions import cartesian_ptp, circular, joint_ptp, linear
from nova.cell import virtual_controller
from nova.types import MotionSettings, Pose


# Configure the robot program
@nova.program(
    id="start_here",  # Unique identifier of the program. If not provided, the function name will be used.
    name="Start Here",  # Readable name of the program
    # viewer=nova.viewers.Rerun(),  # add this line for a 3D visualization
    preconditions=nova.ProgramPreconditions(
        controllers=[
            virtual_controller(
                name="abb-irb-1300", manufacturer=api.models.Manufacturer.ABB, type="abb-1300_14_7"
            )
        ],
        cleanup_controllers=False,
        dataset=ds.local_dataset("abb-irb-1300-demo.json"),
    ),
)
async def start(
    ctx: nova.ProgramContext,
    count: int = Field(
        default=1, ge=1, le=10, description="The number of times to repeat the movement"
    ),
):
    """Run the dataset routine `inspect-30deg` once as an action list and once as the dataset's CommandRoutine."""
    cell = ctx.cell
    controller = await cell.controller("abb-irb-1300")
    cycle = ctx.cycle(extra={"app": "visual-studio-code"})

    motion_group = controller[0]
    dataset_routine = ctx.dataset.command_routines["inspect-30deg"]
    # The dataset names the TCP "flange"; the virtual controller calls it differently.
    tcp = (await motion_group.tcp_names())[0]
    home = tuple(dataset_routine.start_joint_position)

    # A motion target is a world pose, so resolve the taught poses with `as_world()`.
    def world(pose_id: str) -> Pose:
        return ctx.dataset.poses[pose_id].as_world()

    normal = MotionSettings(tcp_velocity_limit=100)
    fast = MotionSettings(tcp_velocity_limit=200)

    # 1) The routine as an action list.
    actions = [
        joint_ptp(home, settings=normal),
        cartesian_ptp(world("Base-30deg-0m-Via-1"), settings=normal),
        linear(world("Base-30deg-0m-slot-1"), settings=normal),
        cartesian_ptp(world("Base-30deg-0m-appr-1"), settings=normal),
        circular(
            world("Base-30deg-0m-appr-2"),
            intermediate=world("Base-30deg-0m-arc-via"),
            settings=fast,
        ),
        joint_ptp(home, settings=normal),
    ]

    # 2) The routine as stored in the dataset. Its local pose references are resolved to world
    # poses through the dataset. The dataset's TCP name and partial motion group setup do not
    # match this virtual controller, so take both from the motion group.
    routine = dataset_routine.model_copy(
        update={"tcp": tcp, "motion_group_setup": await motion_group.get_setup(tcp)}
    )

    await cycle.start()

    # Both runs start from the routine's start joint position, so move there first.
    await motion_group.plan_and_execute([joint_ptp(home, settings=normal)], tcp)

    for i in range(count):
        # print(f"Run {i + 1} of {count}: action list")
        # joint_trajectory = await motion_group.plan(actions, tcp, start_joint_position=home)
        # await motion_group.execute(joint_trajectory, tcp, actions=actions)
        # await asyncio.sleep(1)

        print(f"Run {i + 1} of {count}: command routine")
        await motion_group.plan_and_execute(resolve_routine(routine, ctx.dataset))
        await asyncio.sleep(1)

    await cycle.finish()
    print("Movement execution completed!")


if __name__ == "__main__":
    run_program(start)
