"""
Test: Path triggers on a straight 300 mm line in Y.

The robot moves 300 mm in the +Y direction (local frame of its home pose) and
two distance-based path triggers bracket the active zone in the middle:

  home ----[IO ON @ +100 mm]---- 100 mm active zone ----[IO OFF @ -100 mm]---- end

Expected IO trace (watch TRIGGER_IO on the controller signals view):
  start   -> False  (explicit write at home, no trigger)
  +100 mm -> True   (after_distance(100) on the Y move)
  +200 mm -> False  (before_distance(100) on the Y move)
  end     -> returns to home

Switching virtual <-> physical:
- Set ``USE_PHYSICAL_ROBOT`` below. When ``False`` a virtual KUKA is provisioned.
- Set ``TRIGGER_IO`` to a digital output that exists on your controller.

Run:
    PYTHONPATH=. uv run python examples/path_triggers_line_test.py
"""

import nova
from nova import api, run_program
from nova.actions import after_distance, before_distance, io_write, joint_ptp, linear
from nova.cell import kuka_controller, virtual_controller
from nova.config import CELL_NAME
from nova.types import MotionSettings, Pose

# --- Controller switch -------------------------------------------------------
USE_PHYSICAL_ROBOT = False

CONTROLLER_NAME = "kuka"
TRIGGER_IO = "OUT#1"

virt_controller = virtual_controller(
    name=CONTROLLER_NAME, manufacturer=api.models.Manufacturer.KUKA, type="kuka-kr16_r2010_2"
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
    id="path_triggers_line_test",
    name="Path Triggers – Line Test",
    # viewer=nova.viewers.Rerun(),  # uncomment for a 3D visualization
    preconditions=nova.ProgramPreconditions(
        controllers=[selected_controller], cleanup_controllers=False
    ),
)
async def main(ctx: nova.ProgramContext) -> None:
    cell = ctx.cell
    controller = await cell.controller(CONTROLLER_NAME)
    motion_group = controller[0]

    if USE_PHYSICAL_ROBOT:
        await ctx.nova.api.controller_api.set_default_mode(
            cell=CELL_NAME,
            controller=CONTROLLER_NAME,
            mode=api.models.SettableRobotSystemMode.MODE_CONTROL,
        )

    speed = MotionSettings(tcp_velocity_limit=50)

    tcp = (await motion_group.tcp_names())[0]
    home_joints = await motion_group.joints()
    home_pose = await motion_group.tcp_pose(tcp)

    # End point: 300 mm in the +Y direction of the home pose's local frame.
    end_pose = home_pose @ Pose((0, 300, 0, 0, 0, 0))

    actions = [
        # Go to home, make sure the output starts low.
        joint_ptp(home_joints, settings=speed),
        io_write(TRIGGER_IO, False),
        # Turn ON 100 mm after leaving home  (anchored to the home → end segment).
        io_write(TRIGGER_IO, True, at=after_distance(100)),
        # Turn OFF 100 mm before reaching the end point (same segment, counting back from end).
        io_write(TRIGGER_IO, False, at=before_distance(100)),
        # Straight 300 mm linear move in +Y.
        linear(end_pose, settings=speed),
        # Return to home.
        joint_ptp(home_joints, settings=speed),
    ]

    print("Planning...")
    trajectory = await motion_group.plan(actions, tcp)

    print(f"Executing: 300 mm line in Y | IO ON @ +100 mm, IO OFF @ -100 mm | watch {TRIGGER_IO}")
    await motion_group.execute(trajectory, tcp, actions=actions)
    print("Done.")


if __name__ == "__main__":
    run_program(main)
