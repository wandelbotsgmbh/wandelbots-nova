"""Run a pre-recorded synchronized trajectory with MultiMotionGroup.

``MultiMotionGroup`` is the single primitive for several motion groups — like
``MotionGroup`` for one. Here it only executes: a trajectory recorded for a robot
and a positioner is run through the IO barrier with ``MultiMotionGroup.execute``,
no planning involved. See ``multi_motion_group_planning.py`` for the
``plan_and_execute`` counterpart.
"""

from pathlib import Path

import nova
from nova import api, run_program
from nova.actions import jnt
from nova.cell import GroupArgs, MultiMotionGroup, virtual_controller

CONTROLLER = "kuka"
ROBOT, POSITIONER = f"0@{CONTROLLER}", f"1@{CONTROLLER}"
TCPS = {ROBOT: "1", POSITIONER: "0"}

SYNC_IO_ID = "OUT#1"

_HERE = Path(__file__).parent


def first_sample(group: str) -> list[float]:
    return load_trajectory().joint_positions_by_motion_group_key[group][0]


def load_trajectory() -> api.models.MultiJointTrajectory:
    """One trajectory for both motion groups: per-group joint samples over a
    single shared times/locations parameterization, which is what makes equal
    location mean the same instant on both paths."""
    path = _HERE / "multi_motion_group_trajectory.json"
    return api.models.MultiJointTrajectory.model_validate_json(path.read_text())


@nova.program(
    name="multi_motion_group",
    preconditions=nova.ProgramPreconditions(
        controllers=[
            virtual_controller(
                name=CONTROLLER,
                manufacturer=api.models.Manufacturer.KUKA,
                type="kuka-kr210_r2700_2",
                controller_config_json=(_HERE / "multi_motion_group_controller.json").read_text(),
                # Spawn the robot where the trajectory starts; ``position`` places
                # the first motion group only, so the positioner is moved below.
                position=[*first_sample(ROBOT), 0.0],
            )
        ]
    ),
)
async def multi_motion_group_trajectory(ctx: nova.ProgramContext):
    """Example of synchronized trajectory execution with two motion groups (robot and positioner)."""
    controller = await ctx.nova.cell().controller(CONTROLLER)
    groups = {name: controller.motion_group(name) for name in TCPS}

    trajectory = load_trajectory()
    samples = trajectory.joint_positions_by_motion_group_key
    for name, group in groups.items():
        # A trajectory can only be executed from its own first sample.
        await group.plan_and_execute([jnt(samples[name][0])], tcp=TCPS[name])

    ensemble = (
        MultiMotionGroup.builder(groups).sync_on_io(SYNC_IO_ID, controller=CONTROLLER).build()
    )
    await ensemble.execute(
        trajectory, groups={name: GroupArgs(tcp=tcp) for name, tcp in TCPS.items()}
    )


if __name__ == "__main__":
    run_program(multi_motion_group_trajectory)
