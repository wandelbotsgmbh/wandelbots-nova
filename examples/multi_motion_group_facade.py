"""Plan and execute one synchronized ensemble through the MultiMotionGroup facade.

``MultiMotionGroup`` is to several motion groups what ``MotionGroup`` is to one:
a single object with ``plan`` / ``execute`` / ``plan_and_execute``. Here it plans
a collision-free move for a robot and a positioner at once (the multi-motion-group
RRT endpoint), fires an output between the two moves, and runs both groups
synchronized through an IO barrier — all in one ``plan_and_execute`` call, with
the same action list feeding both the plan and the execution.

Contrast ``multi_motion_group.py``, which executes a *pre-recorded* trajectory
through ``TrajectoryExecutor`` directly; this one plans live and adds IO.
"""

from pathlib import Path

import nova
from nova.actions import io_write, jnt, multi_collision_free
from nova.cell import MultiMotionGroup, virtual_controller

CONTROLLER = "kuka"
ROBOT, POSITIONER = f"0@{CONTROLLER}", f"1@{CONTROLLER}"
TCPS = {ROBOT: "1", POSITIONER: "0"}

# The barrier IO that releases both groups at the same instant, and an output the
# plan flips between the two synchronized moves (fired on the controller clock,
# anchored to the motion boundary — location 1, after the first move).
SYNC_IO_ID = "OUT#1"
STAGE_DONE_IO_ID = "OUT#2"

_HERE = Path(__file__).parent


def recorded() -> nova.api.models.MultiJointTrajectory:
    """Two known-good joint configs per group, reused here as plan endpoints so the
    example needs no live poses: sample 0 is the start, the last sample the target."""
    path = _HERE / "multi_motion_group_trajectory.json"
    return nova.api.models.MultiJointTrajectory.model_validate_json(path.read_text())


def sample(group: str, index: int) -> list[float]:
    return recorded().joint_positions_by_motion_group_key.root[group].root[index].root


@nova.program(
    name="multi_motion_group_facade",
    preconditions=nova.ProgramPreconditions(
        controllers=[
            virtual_controller(
                name=CONTROLLER,
                manufacturer=nova.api.models.Manufacturer.KUKA,
                type="kuka-kr210_r2700_2",
                controller_config_json=(_HERE / "multi_motion_group_controller.json").read_text(),
                # Spawn the robot where the plan starts; ``position`` places the
                # first motion group only, so the positioner is moved below.
                position=[*sample(ROBOT, 0), 0.0],
            )
        ]
    ),
)
async def multi_motion_group_facade(ctx: nova.ProgramContext):
    """Plan + synchronized execute of two motion groups via MultiMotionGroup."""
    controller = await ctx.nova.cell().controller(CONTROLLER)
    groups = {name: controller.motion_group(name) for name in TCPS}

    # Bring each group to the shared start config before the synchronized run.
    for name, group in groups.items():
        await group.plan_and_execute([jnt(sample(name, 0))], tcp=TCPS[name])

    ensemble = (
        MultiMotionGroup.builder(groups).sync_on_io(SYNC_IO_ID, controller=CONTROLLER).build()
    )

    start = {name: tuple(sample(name, 0)) for name in TCPS}
    target = {name: tuple(sample(name, -1)) for name in TCPS}

    # One call plans both collision-free segments (RRT), lays the IO write onto
    # the seam between them, and runs the ensemble synchronized through the barrier.
    await ensemble.plan_and_execute(
        [
            multi_collision_free(target),
            io_write(STAGE_DONE_IO_ID, True, device_id=CONTROLLER),
            multi_collision_free(start),
        ],
        tcp=TCPS,
    )
    print("synchronized plan_and_execute finished; stage-done output was set at the seam")


if __name__ == "__main__":
    nova.run_program(multi_motion_group_facade)
