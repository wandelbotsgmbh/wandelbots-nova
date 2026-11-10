"""Pick the cube up and put it on the marker, by hand rather than by policy.

This is the scripted version of the task the choreo3 checkpoints learn. It is
here for two reasons: to prove a cell before a policy ever runs on it, and to
show what the policy is replacing. Same robot, same `umi_corrected` TCP, same
gripper IO — the only difference is that every pose below was written out
instead of predicted.

Run from the repository root. Review the arguments first:

    PYTHONPATH=. uv run --extra novapolicy \\
        python novapolicy/examples/pick_and_place_umi_ur10e.py --help

Add ``--yes`` only when you are ready to execute robot motion:

    PYTHONPATH=. uv run --extra novapolicy \\
        python novapolicy/examples/pick_and_place_umi_ur10e.py \\
        --nova-api http://<nova-host> \\
        --simulation http://<sim-host>:8011 \\
        --yes

This attaches to an instance and a simulation that are already running; it
starts neither. Set the cell up first with::

    ./novapolicy/examples/setup_cell.sh http://<nova-host> umi

which installs the UR10e and the `umi_corrected` TCP this example needs.

The cube's position is read from the simulation's Omniverse-compatible API when
it is reachable. Point ``--simulation`` at a host without one and pass
``--cube-x`` / ``--cube-y`` / ``--table`` instead.
"""

from __future__ import annotations

import argparse
import asyncio
import math
from typing import TYPE_CHECKING

import httpx
from scipy.spatial.transform import Rotation

from nova import Nova, NovaConfig
from nova.actions import cartesian_ptp, io_write, joint_ptp, linear, wait
from nova.types import MotionSettings, Pose

if TYPE_CHECKING:
    from nova.actions import Action

CUBE = "/World/cell/universalrobots_ur10e/objects/cube"

# Joint pose the arm starts and finishes at, tool pointing down.
HOME = (0.0, -1.658, 1.590, -1.569, -1.571, 0.0)

# The gripper is one boolean IO. True closes it — that comes from the scene's
# ActionGraph, not from convention.
GRIP, RELEASE = True, False

# Centre of the yellow marker, in the robot's base frame.
GOAL_XY = (-0.6, -420.2)

# The TCP sits above and slightly behind the point between the fingertips, so
# every target is offset by this much in the tool frame.
JAW_OFFSET = (2.3, -0.8, 9.1)

GRASP_HEIGHT = 15.0  # fingertips above the cube's base when gripping, mm
APPROACH = 150.0  # how high above a target to arrive and leave, mm
TRAVEL = MotionSettings(tcp_velocity_limit=150.0)
CAREFUL = MotionSettings(tcp_velocity_limit=25.0)


def above(x: float, y: float, z: float, height: float) -> Pose:
    """A tool-down pose that puts the fingertips at ``(x, y, z + height)``."""
    rotation = Rotation.from_euler("z", 0.0) * Rotation.from_euler("x", math.pi)
    shift = rotation.apply(JAW_OFFSET)
    return Pose(
        x - float(shift[0]),
        y - float(shift[1]),
        z + height - float(shift[2]),
        *(float(v) for v in rotation.as_rotvec()),
    )


def pick_and_place(x: float, y: float, table: float, gripper_io: str) -> list[Action]:
    """The whole motion: home, grasp at ``(x, y)``, place on the marker, home."""
    return [
        joint_ptp(HOME, TRAVEL),
        io_write(gripper_io, value=RELEASE),
        # Arrive above the cube, descend slowly onto it, and leave slowly: at
        # travel speed the jaws brush a 60 mm cube across the table.
        cartesian_ptp(above(x, y, table, APPROACH), TRAVEL),
        linear(above(x, y, table, GRASP_HEIGHT), CAREFUL),
        io_write(gripper_io, value=GRIP),
        wait(2.0),
        linear(above(x, y, table, APPROACH), CAREFUL),
        # A straight line at a fixed orientation: a point-to-point move may
        # swing the wrist, and a cube held by the fingertips is dropped by that.
        linear(above(*GOAL_XY, table, APPROACH), TRAVEL),
        linear(above(*GOAL_XY, table, GRASP_HEIGHT), CAREFUL),
        io_write(gripper_io, value=RELEASE),
        wait(2.0),
        linear(above(*GOAL_XY, table, APPROACH), CAREFUL),
        joint_ptp(HOME, TRAVEL),
    ]


async def cube_position(api: httpx.AsyncClient) -> tuple[float, float, float]:
    """Where the cube is now, in the robot's base frame, in millimetres."""
    response = await api.get(
        "/prims/poses", params={"prim_path": CUBE, "rotation_type": "cartesian"}
    )
    response.raise_for_status()
    pose = response.json()["pose"]
    return float(pose[0]), float(pose[1]), float(pose[2])


async def run(args: argparse.Namespace) -> None:
    base = f"{args.simulation.rstrip('/')}/omniservice/api/v2"
    async with httpx.AsyncClient(base_url=base, timeout=10.0) as api:
        if args.cube_x is None:
            await api.patch("/stage/simulation/timeline/play")
            x, y, table = await cube_position(api)
        else:
            x, y, table = args.cube_x, args.cube_y, args.table
        print(f"cube at x={x:.1f} y={y:.1f} z={table:.1f}")

        nova = Nova(NovaConfig(host=args.nova_api, verify_ssl=False))
        try:
            controller = await nova.cell(args.cell).controller(args.controller)
            async with controller[0] as motion_group:
                await motion_group.plan_and_execute(
                    pick_and_place(x, y, table, args.gripper_io), args.tcp
                )
        finally:
            await nova.close()

        if args.cube_x is None:
            final = await cube_position(api)
            off = math.hypot(final[0] - GOAL_XY[0], final[1] - GOAL_XY[1])
            print(f"cube now at x={final[0]:.1f} y={final[1]:.1f} — {off:.0f} mm from the marker")


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--nova-api", required=True, help="NOVA instance base URL")
    parser.add_argument(
        "--simulation",
        required=True,
        help="Simulation host serving /omniservice and /webrtc-streamer, e.g. http://10.2.0.90:8011",
    )
    parser.add_argument("--cell", default="cell")
    parser.add_argument("--controller", default="ur10e")
    parser.add_argument(
        "--tcp",
        default="umi_corrected",
        help="Frame the demonstrations were recorded in, not the cell's Flange",
    )
    parser.add_argument("--gripper-io", default="digital_out[0]")
    parser.add_argument(
        "--cube-x",
        type=float,
        help="Cube position in mm; read from the simulation when omitted",
    )
    parser.add_argument("--cube-y", type=float)
    parser.add_argument("--table", type=float, help="Cube base height in mm")
    parser.add_argument("--yes", action="store_true", help="Confirm execution")
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    if not args.yes:
        raise SystemExit(
            "This example moves the robot. Re-run with --yes after checking the arguments."
        )
    supplied = [args.cube_x, args.cube_y, args.table]
    if any(v is not None for v in supplied) and any(v is None for v in supplied):
        raise SystemExit("Pass --cube-x, --cube-y and --table together, or none of them.")
    asyncio.run(run(args))


if __name__ == "__main__":
    main()
