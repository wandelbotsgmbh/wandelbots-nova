"""
Example: PID jogging with jog_joints() and jog_tcp() on two UR10e robots.

Demonstrates four modes:
1. Single-arm joint jogging
2. Single-arm TCP jogging
3. Dual-arm joint jogging
4. Dual-arm TCP jogging

Prerequisites:
    NOVA_API=http://<instance-ip>

Run:
    PYTHONPATH=. python policy/examples/jogging_dualarm.py
"""

import math

from policy import EmergencyStopError, MotionError, jog_joints, jog_tcp

import nova
from nova import api, run_program
from nova.cell import virtual_controller
from nova.program import ProgramPreconditions
from nova.types import Pose

HOME_LEFT = [1.047, -0.698, 1.745, -3.142, 0.873, 2.094]
HOME_RIGHT = [-1.047, -2.356, -1.745, 0.0, -0.873, -2.094]

DURATION = 5.0  # seconds per demo
HZ = 30


# ---------------------------------------------------------------------------
# 1) Single-arm joint jogging
# ---------------------------------------------------------------------------


async def demo_single_joint(mg):
    """Oscillate joint 4 on a single arm."""
    print("\n=== Single-arm joint jogging ===")
    t = 0.0
    async with jog_joints(mg) as jogger:
        async for state in jogger:
            target = list(state.joints)
            target[3] += 0.3 * math.sin(t * 2 * math.pi * 0.5)
            jogger.target = target
            t += 1 / HZ
            if t > DURATION:
                break
    print(f"  {int(t * HZ)} steps")


# ---------------------------------------------------------------------------
# 2) Single-arm TCP jogging
# ---------------------------------------------------------------------------


async def demo_single_tcp(mg, tcp_name: str):
    """Trace a circle in XY with the TCP."""
    print("\n=== Single-arm TCP jogging ===")
    t = 0.0
    async with jog_tcp(mg, tcp=tcp_name) as jogger:
        # Read the starting pose from the first state
        start_pose: Pose | None = None
        async for state in jogger:
            if start_pose is None:
                start_pose = state.pose
            radius = 30.0  # mm
            freq = 0.3  # Hz
            target = Pose(
                start_pose.position[0] + radius * math.cos(2 * math.pi * freq * t),
                start_pose.position[1] + radius * math.sin(2 * math.pi * freq * t),
                start_pose.position[2],
                start_pose.orientation[0],
                start_pose.orientation[1],
                start_pose.orientation[2],
            )
            jogger.target = target
            t += 1 / HZ
            if t > DURATION:
                break
    print(f"  {int(t * HZ)} steps")


# ---------------------------------------------------------------------------
# 3) Dual-arm joint jogging
# ---------------------------------------------------------------------------


async def demo_dual_joint(mg1, mg2):
    """Mirror-symmetric oscillation on two arms."""
    print("\n=== Dual-arm joint jogging ===")
    t = 0.0
    async with jog_joints([mg1, mg2]) as jogger:
        async for states in jogger:
            s1, s2 = states[mg1], states[mg2]
            t1 = list(s1.joints)
            t2 = list(s2.joints)

            wave = 0.3 * math.sin(t * 2 * math.pi * 0.5)
            t1[3] += wave
            t2[3] -= wave  # mirror

            jogger.target = {mg1: t1, mg2: t2}
            t += 1 / HZ
            if t > DURATION:
                break
    print(f"  {int(t * HZ)} steps")


# ---------------------------------------------------------------------------
# 4) Dual-arm TCP jogging
# ---------------------------------------------------------------------------


async def demo_dual_tcp(mg1, mg2, tcp1: str, tcp2: str):
    """Both TCPs trace circles — left clockwise, right counter-clockwise."""
    print("\n=== Dual-arm TCP jogging ===")
    t = 0.0
    async with jog_tcp({mg1: tcp1, mg2: tcp2}) as jogger:
        start1: Pose | None = None
        start2: Pose | None = None
        async for states in jogger:
            if start1 is None:
                start1 = states[mg1].pose
                start2 = states[mg2].pose

            radius = 30.0
            freq = 0.3
            angle = 2 * math.pi * freq * t

            target1 = Pose(
                start1.position[0] + radius * math.cos(angle),
                start1.position[1] + radius * math.sin(angle),
                start1.position[2],
                *start1.orientation,
            )
            target2 = Pose(
                start2.position[0] + radius * math.cos(-angle),
                start2.position[1] + radius * math.sin(-angle),
                start2.position[2],
                *start2.orientation,
            )
            jogger.target = {mg1: target1, mg2: target2}
            t += 1 / HZ
            if t > DURATION:
                break
    print(f"  {int(t * HZ)} steps")


# ---------------------------------------------------------------------------
# Program
# ---------------------------------------------------------------------------


@nova.program(
    id="jogging_dualarm",
    name="Jogging Dual-Arm Demo",
    description="Demonstrates jog_joints() and jog_tcp() on two UR10e robots.",
    preconditions=ProgramPreconditions(
        controllers=[
            virtual_controller(
                name="ur5e-left",
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type="universalrobots-ur5e",
            ),
            virtual_controller(
                name="ur5e-right",
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type="universalrobots-ur5e",
            ),
        ],
        cleanup_controllers=False,
    ),
)
async def jogging_dualarm(ctx: nova.ProgramContext):
    cell = ctx.nova.cell()
    mg1 = (await cell.controller("ur5e-left"))[0]
    mg2 = (await cell.controller("ur5e-right"))[0]
    tcp1 = (await mg1.tcp_names())[0]
    tcp2 = (await mg2.tcp_names())[0]

    try:
        await demo_single_joint(mg1)
        await demo_single_tcp(mg1, tcp1)
        await demo_dual_joint(mg1, mg2)
        await demo_dual_tcp(mg1, mg2, tcp1, tcp2)
        print("\nAll demos complete.")
    except MotionError as e:
        print(f"\nMotion error (joint limit / collision): {e}")
    except EmergencyStopError as e:
        print(f"\nEmergency stop on controller '{e.controller_id}': {e.safety_state}")


if __name__ == "__main__":
    run_program(jogging_dualarm)
