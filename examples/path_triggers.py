"""
Example: Path triggers ("Bahnschaltpunkte") for IO writes between motions.

A path trigger lets you fire an ``io_write`` at a precise point on the planned
path *between* two motion actions, instead of only at the motion-command
boundaries. The trigger is anchored to the action's position in the action list
(the motion segment between the previous and the next motion) and addresses a
point *within* that segment using one of three modes:

- ``at_path(fraction)``        -> 0.0 = previous motion, 1.0 = next motion
- ``after_time(seconds)``      -> seconds after the previous motion
- ``before_time(seconds)``     -> seconds before the next motion
- ``after_distance(mm)``       -> mm of TCP travel after the previous motion
- ``before_distance(mm)``      -> mm of TCP travel before the next motion

Time- and distance-based triggers are resolved against the planned trajectory
during ``execute`` (distance uses the planned Cartesian TCP path length). Values
that overshoot the anchor segment are clamped to its boundary with a warning.

This example pulses a single controller digital output at several points along a
square-ish path so you can observe the output toggling as the robot moves.

Switching virtual <-> physical:
- Set ``USE_PHYSICAL_ROBOT`` below. When ``False`` a virtual KUKA is provisioned
  and you can run this against any NOVA instance. When ``True`` the physical
  ``kuka_controller`` connection settings are used instead.
- Adjust ``TRIGGER_IO`` to a digital output that exists on your controller.

Prerequisites:
- A NOVA instance (see .env / NOVA_API, NOVA_ACCESS_TOKEN)
- Run this example script:
    PYTHONPATH=. uv run python examples/path_triggers.py
"""

import nova
from nova import api, run_program
from nova.actions import (
    after_distance,
    after_time,
    at_path,
    before_distance,
    before_time,
    cartesian_ptp,
    io_write,
    joint_ptp,
)
from nova.cell import kuka_controller, virtual_controller
from nova.config import CELL_NAME
from nova.types import MotionSettings, Pose

# --- Controller switch -------------------------------------------------------
# Flip this to run against a real robot. The rest of the program is identical.
USE_PHYSICAL_ROBOT = False

CONTROLLER_NAME = "kuka"

# Digital output that gets pulsed by the path triggers. Adjust to an output that
# exists on your controller (KUKA controllers expose e.g. "OUT#1").
TRIGGER_IO = "OUT#1"

virt_controller = virtual_controller(
    name=CONTROLLER_NAME, manufacturer=api.models.Manufacturer.KUKA, type="kuka-kr240_r2900"
)

phys_controller = kuka_controller(
    name=CONTROLLER_NAME,
    controller_ip="192.168.101.131",
    controller_port=54600,
    rsi_server_ip="192.168.102.130",
    rsi_server_port=30152,
)

selected_controller = phys_controller if USE_PHYSICAL_ROBOT else virt_controller


@nova.program(
    id="path_triggers",
    name="Path Triggers",
    # viewer=nova.viewers.Rerun(),  # uncomment for a 3D visualization
    preconditions=nova.ProgramPreconditions(
        controllers=[selected_controller], cleanup_controllers=False
    ),
)
async def main(ctx: nova.ProgramContext) -> None:
    cell = ctx.cell
    controller = await cell.controller(CONTROLLER_NAME)
    motion_group = controller[0]

    # A physical KUKA needs to be in control mode before it will move.
    if USE_PHYSICAL_ROBOT:
        await ctx.nova.api.controller_api.set_default_mode(
            cell=CELL_NAME,
            controller=CONTROLLER_NAME,
            mode=api.models.SettableRobotSystemMode.MODE_CONTROL,
        )

    normal = MotionSettings(tcp_velocity_limit=100)
    fast = MotionSettings(tcp_velocity_limit=250)

    tcp = (await motion_group.tcp_names())[0]
    home_joints = await motion_group.joints()
    home_pose = await motion_group.tcp_pose(tcp)

    # Four corners of a square in the home pose's local frame.
    p1 = home_pose @ Pose((150, 0, 0, 0, 0, 0))
    p2 = home_pose @ Pose((150, 150, 0, 0, 0, 0))
    p3 = home_pose @ Pose((0, 150, 0, 0, 0, 0))

    # Each io_write is anchored to the motion segment it is placed in (the move
    # arriving at the motion that follows it in this list).
    actions = [
        joint_ptp(home_joints, settings=normal),
        # Make sure the output starts low at the home boundary (no trigger).
        io_write(TRIGGER_IO, False),
        # Fire 0.3 s into the home -> p1 move.
        io_write(TRIGGER_IO, True, at=after_time(0.3)),
        cartesian_ptp(p1, settings=fast),
        # Drop the output 50 mm into the p1 -> p2 move.
        io_write(TRIGGER_IO, False, at=after_distance(50)),
        cartesian_ptp(p2, settings=fast),
        # Raise it again 50 mm before reaching p3.
        io_write(TRIGGER_IO, True, at=before_distance(50)),
        cartesian_ptp(p3, settings=fast),
        # Drop it exactly halfway through the p3 -> home move.
        io_write(TRIGGER_IO, False, at=at_path(0.5)),
        # And raise it 0.3 s before arriving back home.
        io_write(TRIGGER_IO, True, at=before_time(0.3)),
        joint_ptp(home_joints, settings=normal),
    ]

    print("Planning trajectory with path-triggered IO writes...")
    trajectory = await motion_group.plan(actions, tcp)

    print("Executing... watch", TRIGGER_IO, "toggle along the path.")
    await motion_group.execute(trajectory, tcp, actions=actions)
    print("Done.")


if __name__ == "__main__":
    run_program(main)
