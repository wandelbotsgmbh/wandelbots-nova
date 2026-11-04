"""
Run a CommandRoutine stored in a Dataset on a virtual KUKA KR16.

The program's preconditions load the Dataset `pick-and-place` from the NOVA instance. The routine
`pick-and-place` is taken from it and resolved by NOVA: its references to the dataset's poses and
frames become inline poses in the world frame, so it runs without the dataset. NOVA plans and runs
the whole routine with its own context. `execute` returns once NOVA has accepted the run, not when
the robot is done.

See `docs/command_routines.md` for what is supported.
"""

import nova
from nova import api, run_program
from nova import datasets as ds
from nova.cell import virtual_controller
from nova.command_routines import CommandRoutine

DATASET = "pick-and-place"
ROUTINE = "pick-and-place"


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
        dataset=ds.remote_dataset(DATASET),
        cleanup_controllers=False,
    ),
)
async def start(ctx: nova.ProgramContext):
    """Run a routine of the program's dataset."""
    assert ctx.dataset is not None, "This program requires its dataset to be loaded."
    controller = await ctx.cell.controller("kuka-kr16-r2010")
    motion_group = controller[0]
    tcp = (await motion_group.tcp_names())[0]

    # The routine as stored in the dataset, extracted from the dataset of the preconditions.
    stored = ctx.dataset.command_routines[ROUTINE]

    # Resolve it, so that its dataset poses become world poses. The TCP and motion group setup
    # the dataset was taught with do not match this virtual controller, so take both from it.
    resolved = await CommandRoutine.resolve(ctx.nova, ctx.dataset.id, stored.command_routine)
    routine = resolved.model_copy(
        update={"tcp": tcp, "motion_group_setup": await motion_group.get_setup(tcp)}
    )

    print("Starting routine...")
    run = await routine.execute(motion_group)
    print(f"NOVA runs the routine now: run {run.run} is {run.state.value}.")


if __name__ == "__main__":
    run_program(start)
