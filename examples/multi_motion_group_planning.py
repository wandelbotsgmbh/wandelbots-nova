"""Plan and execute one synchronized ensemble with MultiMotionGroup.

``MultiMotionGroup`` is to several motion groups what ``MotionGroup`` is to one:
a single object with ``plan`` / ``execute`` / ``plan_and_execute``. Here it plans
a collision-free move for a robot and a positioner at once (the multi-motion-group
RRT endpoint), fires an output between the two moves, and runs both groups
synchronized through an IO barrier — all in one ``plan_and_execute`` call, with
the same action list feeding both the plan and the execution.

Contrast ``multi_motion_group.py``, which runs a *pre-recorded* trajectory with
``MultiMotionGroup.execute``; this one plans live and adds IO.
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

# Fixed joint configs the plan moves between — robot (6 joints) and positioner
# (2 joints). The move is planned collision-free, so no live poses are needed.
START = {ROBOT: (0.267, -1.497, 2.035, -1.157, 0.5, 1.133), POSITIONER: (-0.167, 3.06)}
TARGET = {ROBOT: (0.0, -1.564, 1.57, 0.0, 1.57, 0.0), POSITIONER: (0.0, 0.0)}

_HERE = Path(__file__).parent


@nova.program(
    name="multi_motion_group_planning",
    preconditions=nova.ProgramPreconditions(
        controllers=[
            virtual_controller(
                name=CONTROLLER,
                manufacturer=nova.api.models.Manufacturer.KUKA,
                type="kuka-kr210_r2700_2",
                controller_config_json=(_HERE / "multi_motion_group_controller.json").read_text(),
                # Spawn the robot where the plan starts; ``position`` places the
                # first motion group only, so the positioner is moved below.
                position=[*START[ROBOT], 0.0],
            )
        ]
    ),
)
async def multi_motion_group_planning(ctx: nova.ProgramContext):
    """Plan + synchronized execute of two motion groups via MultiMotionGroup."""
    controller = await ctx.nova.cell().controller(CONTROLLER)
    groups = {name: controller.motion_group(name) for name in TCPS}

    # Bring each group to the shared start config before the synchronized run.
    for name, group in groups.items():
        await group.plan_and_execute([jnt(START[name])], tcp=TCPS[name])

    ensemble = (
        MultiMotionGroup.builder(groups).sync_on_io(SYNC_IO_ID, controller=CONTROLLER).build()
    )

    # One call plans both collision-free segments (RRT), lays the IO write onto
    # the seam between them, and runs the ensemble synchronized through the barrier.
    await ensemble.plan_and_execute(
        [
            multi_collision_free(TARGET),
            io_write(STAGE_DONE_IO_ID, True, device_id=CONTROLLER),
            multi_collision_free(START),
        ],
        tcp=TCPS,
    )
    print("synchronized plan_and_execute finished; stage-done output was set at the seam")


if __name__ == "__main__":
    nova.run_program(multi_motion_group_planning)
