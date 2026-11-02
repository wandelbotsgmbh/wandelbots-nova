"""
Run a CommandRoutine on a virtual KUKA KR16, with poses taught in the dataset of `datasets.py`.

The routine moves to the dataset poses `pick` and `place` and back, switches an output between the
motions and waits one second. NOVA plans and runs the whole routine with its own context.
`plan_and_execute` returns once NOVA has accepted the run, not when the robot is done.

`pick` and `place` are taught in the world frame, so they can be used as the routine's inline
poses as they are. A routine stored in a Dataset on the NOVA instance refers to its poses by id
instead; resolve it with `ds.resolve_command_routine` before running it.

See `docs/command_routines.md` for what is supported.
"""

import nova
from nova import api, run_program
from nova import datasets as ds
from nova.cell import virtual_controller
from nova.command_routines import (
    command_routine,
    move_cartesian_ptp,
    set_io,
    wait_for_time,
    with_settings,
)

# Output that exists on the virtual KUKA controller.
OUTPUT = "OUT#1"


@nova.program(
    id="command_routine",
    name="Command routine",
    preconditions=nova.ProgramPreconditions(
        controllers=[
            virtual_controller(
                name="kuka-kr16-r2010",
                manufacturer=api.models.Manufacturer.KUKA,
                type="kuka-kr16_r2010_2",
            )
        ],
        dataset=ds.local_dataset("example_dataset.json"),
        cleanup_controllers=False,
    ),
)
async def start(ctx: nova.ProgramContext):
    """Run a routine with explicit motions, set_io and a wait."""
    controller = await ctx.cell.controller("kuka-kr16-r2010")
    motion_group = controller[0]
    tcp = (await motion_group.tcp_names())[0]

    def taught(pose_id: str) -> api.models.InlinePose:
        pose = ctx.dataset.poses[pose_id]
        return api.models.InlinePose(
            pose=pose.pose.to_api_model(),
            frame=pose.frame,
            tcp=pose.tcp,
            kinematic_configuration=pose.kinematic_configuration,
        )

    slow = api.models.MotionSettings(
        limits_override=api.models.LimitsOverride(tcp_velocity_limit=50)
    )

    routine = command_routine(
        "pick-and-place",
        tcp=tcp,
        motion_group_setup=await motion_group.get_setup(tcp),
        start_joint_position=list(await motion_group.joints()),
        commands=[
            move_cartesian_ptp(taught("pick")),
            set_io(OUTPUT, True),  # between two motions: written by the controller
            *with_settings(slow, [move_cartesian_ptp(taught("place"))]),
            wait_for_time(1000),  # the robot holds for one second
            set_io(OUTPUT, False),
            move_cartesian_ptp(taught("pick")),
        ],
    )

    print("Starting routine...")
    run = await motion_group.plan_and_execute(routine)
    print(f"NOVA runs the routine now: run {run.run} is {run.state.value}.")


if __name__ == "__main__":
    run_program(start)
