"""
Run a CommandRoutine of a dataset on a virtual KUKA KR16.

NOVA plans and runs the whole routine: motions, set_io (with and without trigger), waits,
pause_on_io and markers. A routine stored in a dataset is resolved by NOVA first, so its poses
become world poses and it runs without the dataset.

This example uses the `default` dataset seeded on the NOVA instance (the same content as
`example_dataset.json`). Its `startup-routine` only waits, so the example gives it this robot's
context and appends motions to the dataset poses `pick` and `place`, plus two `set_io`
commands: one at a motion boundary and one with a path-fraction trigger.

See `docs/command_routines.md` for what is supported.
"""

import nova
from nova import api, run_program
from nova import command_routines as cr
from nova import datasets as ds
from nova.cell import virtual_controller
from nova.command_routines import at_path_fraction, move_cartesian_ptp, set_io, with_settings

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
        dataset=ds.remote_dataset("default"),
        cleanup_controllers=False,
    ),
)
async def start(ctx: nova.ProgramContext):
    """Run a dataset routine with a wait, explicit motions and set_io, with and without trigger."""
    controller = await ctx.cell.controller("kuka-kr16-r2010")
    motion_group = controller[0]
    tcp = (await motion_group.tcp_names())[0]
    home = tuple(await motion_group.joints())

    # `pick` and `place` are taught in world; move to them with this robot's TCP.
    def taught(pose_id: str) -> api.models.InlinePose:
        pose = ctx.dataset.poses[pose_id]
        return api.models.InlinePose(
            pose=pose.as_world().to_api_model(),
            frame=api.models.WorldFrameReference(),
            tcp=tcp,
            kinematic_configuration=pose.kinematic_configuration,
        )

    slow = api.models.MotionSettings(
        limits_override=api.models.LimitsOverride(tcp_velocity_limit=50)
    )

    seed = await ds.resolve_command_routine(ctx.nova, "default", "startup-routine")
    routine = seed.model_copy(
        update={
            "commands": [
                *seed.commands,  # waits for one second
                move_cartesian_ptp(taught("pick")),
                set_io(OUTPUT, True),  # without trigger: at the motion boundary
                *with_settings(slow, [move_cartesian_ptp(taught("place"))]),
                set_io(OUTPUT, False, at=at_path_fraction(0.5)),  # with trigger
                move_cartesian_ptp(taught("pick")),
            ],
            # The seed routine carries no context, so take it from the motion group.
            "motion_group": api.models.MotionGroupReference(id=motion_group.id),
            "tcp": tcp,
            "motion_group_setup": await motion_group.get_setup(tcp),
            "start_joint_position": list(home),
        }
    )

    print("Running routine...")
    await cr.execute(ctx.nova, routine)
    print(f"{OUTPUT} = {await controller.read(OUTPUT)}")


if __name__ == "__main__":
    run_program(start)
