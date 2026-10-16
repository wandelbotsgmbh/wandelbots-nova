"""
Run a CommandRoutine on a virtual KUKA KR16, reusing the dataset of `datasets.py`.

The dataset's `startup-routine` only holds a `wait_for_time` command, which is not supported yet.
This example therefore drops unsupported commands from the dataset routine and appends explicit
motions to the dataset poses `pick` and `place`, plus two `set_io` commands: one at a motion
boundary and one with a path-fraction trigger. It then plans and runs the routine.

Which commands are supported: see `docs/command_routines.md` ("Supported commands" and
"Not supported yet") and `nova/cell/routine_execution.py`.
"""

import nova
from nova import api, run_program
from nova import datasets as ds
from nova.actions import joint_ptp
from nova.cell import virtual_controller
from nova.cell.routine_execution import resolve_routine
from nova.command_routines import (
    at_path_fraction,
    move_cartesian_ptp,
    move_linear,
    set_io,
    with_settings,
)

# Output that exists on the virtual KUKA controller.
OUTPUT = "OUT#1"

# TODO: Remove this filter once the SDK can execute these commands. Until then a routine
# containing one of them is rejected with `UnsupportedRoutineCommand` before the robot moves.
UNSUPPORTED = ("wait_for_time", "wait_for_io", "pause_on_io", "marker", "generated_motion")


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
    """Run a Dataset routine with explicit motions and set_io, with and without trigger."""
    controller = await ctx.cell.controller("kuka-kr16-r2010")
    motion_group = controller[0]
    tcp = (await motion_group.tcp_names())[0]
    home = tuple(await motion_group.joints())

    slow = api.models.MotionSettings(
        limits_override=api.models.LimitsOverride(tcp_velocity_limit=50)
    )

    fast = api.models.MotionSettings(
        limits_override=api.models.LimitsOverride(tcp_velocity_limit=250)
    )

    dataset_routine = ctx.dataset.command_routines["startup-routine"]
    supported = [c for c in dataset_routine.commands if c.type not in UNSUPPORTED]

    routine = dataset_routine.model_copy(
        update={
            "commands": [
                *supported,
                move_cartesian_ptp("pick"),  # dataset poses are resolved through the dataset
                set_io(OUTPUT, True),  # without trigger: at the motion boundary
                *with_settings(slow, [move_linear("place")]),
                set_io(OUTPUT, False, at=at_path_fraction(0.5)),  # with trigger
                *with_settings(fast, [move_cartesian_ptp("pick")]),
            ],
            # The seed routine carries no context, so take it from the motion group.
            "tcp": tcp,
            "motion_group_setup": await motion_group.get_setup(tcp),
            "start_joint_position": list(home),
        }
    )

    print("Running routine...")
    await motion_group.plan_and_execute(resolve_routine(routine, ctx.dataset))
    await motion_group.plan_and_execute([joint_ptp(home)], tcp)
    print(f"{OUTPUT} = {await controller.read(OUTPUT)}")


if __name__ == "__main__":
    run_program(start)
