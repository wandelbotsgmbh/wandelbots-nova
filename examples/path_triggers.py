"""
Example: Path triggers ("Bahnschaltpunkte") for IO writes within a motion.

A path trigger lets you fire an ``io_write`` at a precise point on the planned path
*within* a motion, instead of only at the motion-command boundaries.

Place the ``io_write`` directly *before* the motion it belongs to. Without a trigger it
fires at that boundary, i.e. when the motion starts. With a trigger it fires inside the
motion, measured from the motion's start or back from its target:

- ``after_start(seconds=0.3)``       -> 0.3 s after the motion starts
- ``after_start(millimeters=50)``    -> 50 mm of TCP travel after the motion starts
- ``before_target(seconds=0.3)``     -> 0.3 s before the motion reaches its target
- ``before_target(millimeters=50)``  -> 50 mm of TCP travel before the target
- ``at_path_fraction(0.5)``          -> halfway through the motion (fraction in [0, 1))

``seconds`` also accepts a ``datetime.timedelta``. The trigger objects are the same
``AtTrigger`` types the NOVA command-routine API uses for ``set_io(at=...)``.

Time- and distance-based triggers are resolved against the planned trajectory during
``execute`` (distance uses the planned Cartesian TCP path length). Values that
overshoot the motion are clamped to its boundary with a warning.

This example provisions a virtual KUKA and pulses a single controller digital output
at several points along a square-ish path so you can observe the output toggling as
the robot moves. Adjust ``TRIGGER_IO`` to a digital output that exists on your
controller.

Prerequisites:
- A NOVA instance (see .env / NOVA_API, NOVA_ACCESS_TOKEN)
- Run this example script:
    PYTHONPATH=. uv run python examples/path_triggers.py
"""

from datetime import timedelta

import nova
from nova import api, run_program
from nova.actions import (
    after_start,
    at_path_fraction,
    before_target,
    cartesian_ptp,
    io_write,
    joint_ptp,
)
from nova.cell import virtual_controller
from nova.types import MotionSettings, Pose

CONTROLLER_NAME = "kuka"

# Digital output that gets pulsed by the path triggers. Adjust to an output that
# exists on your controller (KUKA controllers expose e.g. "OUT#1").
TRIGGER_IO = "OUT#1"

virtual_kuka = virtual_controller(
    name=CONTROLLER_NAME, manufacturer=api.models.Manufacturer.KUKA, type="kuka-kr240_r2900"
)


@nova.program(
    id="path_triggers",
    name="Path Triggers",
    # viewer=nova.viewers.Rerun(),  # uncomment for a 3D visualization
    preconditions=nova.ProgramPreconditions(controllers=[virtual_kuka], cleanup_controllers=False),
)
async def main(ctx: nova.ProgramContext) -> None:
    cell = ctx.cell
    controller = await cell.controller(CONTROLLER_NAME)
    motion_group = controller[0]

    normal = MotionSettings(tcp_velocity_limit=100)
    fast = MotionSettings(tcp_velocity_limit=250)

    tcp = (await motion_group.tcp_names())[0]
    home_joints = await motion_group.joints()
    home_pose = await motion_group.tcp_pose(tcp)

    # Four corners of a square in the home pose's local frame.
    p1 = home_pose @ Pose((150, 0, 0, 0, 0, 0))
    p2 = home_pose @ Pose((150, 150, 0, 0, 0, 0))
    p3 = home_pose @ Pose((0, 150, 0, 0, 0, 0))

    # Every io_write belongs to the motion that follows it in this list.
    actions = [
        joint_ptp(home_joints, settings=normal),
        # No trigger: fires at the boundary, right when the home -> p1 move starts.
        io_write(TRIGGER_IO, False),
        # --- home -> p1: raise the output 0.3 s after the move starts.
        io_write(TRIGGER_IO, True, at=after_start(seconds=0.3)),
        cartesian_ptp(p1, settings=fast),
        # --- p1 -> p2: drop it 50 mm into the move.
        io_write(TRIGGER_IO, False, at=after_start(millimeters=50)),
        cartesian_ptp(p2, settings=fast),
        # --- p2 -> p3: raise it 50 mm before reaching p3.
        io_write(TRIGGER_IO, True, at=before_target(millimeters=50)),
        cartesian_ptp(p3, settings=fast),
        # --- p3 -> home: drop it halfway, raise it 0.3 s before arriving home.
        io_write(TRIGGER_IO, False, at=at_path_fraction(0.5)),
        io_write(TRIGGER_IO, True, at=before_target(seconds=timedelta(milliseconds=300))),
        joint_ptp(home_joints, settings=normal),
    ]

    print("Planning trajectory with path-triggered IO writes...")
    trajectory = await motion_group.plan(actions, tcp)

    print("Executing... watch", TRIGGER_IO, "toggle along the path.")
    await motion_group.execute(trajectory, tcp, actions=actions)
    print("Done.")


if __name__ == "__main__":
    run_program(main)
