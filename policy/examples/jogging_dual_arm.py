"""
Example: PID jogging with jog_joints() and jog_tcp() on two UR10e robots.

Demonstrates five modes:
1. Single-arm joint jogging
2. Single-arm joint jogging with chunks (smoother tracking)
3. Single-arm TCP jogging
4. Dual-arm joint jogging
5. Dual-arm TCP jogging

Prerequisites:
    NOVA_API=http://<instance-ip>

Run:
    PYTHONPATH=. python policy/examples/jogging_dual_arm.py
"""

import math
import time

import nova
from nova import api, run_program
from nova.cell import virtual_controller
from nova.program import ProgramPreconditions
from nova.types import Pose
from policy import EmergencyStopError, MotionError, jog_joints, jog_tcp

HOME_LEFT = [0.0, -1.571, 0.0, -1.571, -1.571, 0.0]
HOME_RIGHT = [0.0, -1.571, 0.0, -1.571, 1.571, 0.0]


# ---------------------------------------------------------------------------
# 1) Single-arm joint jogging
# ---------------------------------------------------------------------------


async def demo_single_joint(mg):
    """Oscillate joint 4 for 5 seconds."""
    print("\n=== Single-arm joint jogging ===")
    duration = 5.0  # seconds
    amplitude = 0.15  # radians
    frequency = 0.5  # Hz

    async with jog_joints(mg) as jogger:
        t0 = time.monotonic()
        async for state in jogger:  # yields at ~100Hz
            t = time.monotonic() - t0
            if t >= duration:
                break
            target = list(state.joints)
            target[0] += amplitude * math.sin(2 * math.pi * frequency * t)
            jogger.set_target(target)


# ---------------------------------------------------------------------------
# 2) Single-arm joint jogging with chunks
# ---------------------------------------------------------------------------


async def demo_single_joint_chunked(mg):
    """Same oscillation as demo 1, but sending 8-step chunks.

    Chunks enable interpolation and feedforward velocity between waypoints,
    resulting in smoother motion (see JOGGING.md).
    """
    print("\n=== Single-arm joint jogging (chunked) ===")
    duration = 5.0
    amplitude = 0.15
    frequency = 0.5
    chunk_size = 8
    dt_ms = 33.0  # 33ms between steps ≈ 30fps
    dt_s = dt_ms / 1000.0

    async with jog_joints(mg) as jogger:
        t0 = time.monotonic()
        async for state in jogger:
            t = time.monotonic() - t0
            if t >= duration:
                break
            # Build a chunk of future targets
            base = list(state.joints)
            chunk = []
            for i in range(chunk_size):
                step = list(base)
                step[0] += amplitude * math.sin(2 * math.pi * frequency * (t + i * dt_s))
                chunk.append(step)
            jogger.set_target(chunk, dt_ms=dt_ms)


# ---------------------------------------------------------------------------
# 3) Single-arm TCP jogging
# ---------------------------------------------------------------------------


async def demo_single_tcp(mg, tcp_name: str):
    """Trace a 10mm circle in XZ plane in 2.5 seconds."""
    print("\n=== Single-arm TCP jogging ===")
    duration = 2.5  # seconds for one full circle
    radius = 5.0  # mm

    async with jog_tcp(mg, tcp=tcp_name) as jogger:
        t0 = time.monotonic()
        start_pose = None
        async for state in jogger:  # yields at ~100Hz
            t = time.monotonic() - t0
            if t >= duration:
                break
            if start_pose is None:
                start_pose = state.pose
            angle = 2 * math.pi * (t / duration)  # 0 → 2π over duration
            jogger.set_target(Pose(
                start_pose.position[0] + radius * math.cos(angle),
                start_pose.position[1],
                start_pose.position[2] + radius * math.sin(angle),
                *start_pose.orientation,
            ))


# ---------------------------------------------------------------------------
# 3) Dual-arm joint jogging
# ---------------------------------------------------------------------------


async def demo_dual_joint(mg1, mg2):
    """Mirror-symmetric oscillation on two arms for 5 seconds."""
    print("\n=== Dual-arm joint jogging ===")
    duration = 5.0  # seconds
    amplitude = 0.15  # radians
    frequency = 0.5  # Hz

    async with jog_joints([mg1, mg2]) as jogger:
        t0 = time.monotonic()
        async for states in jogger:  # yields at ~100Hz
            t = time.monotonic() - t0
            if t >= duration:
                break
            wave = amplitude * math.sin(2 * math.pi * frequency * t)
            t1 = list(states[mg1].joints)
            t2 = list(states[mg2].joints)
            t1[0] += wave
            t2[0] -= wave  # mirror
            jogger.set_target({mg1: t1, mg2: t2})


# ---------------------------------------------------------------------------
# 4) Dual-arm TCP jogging
# ---------------------------------------------------------------------------


async def demo_dual_tcp(mg1, mg2, tcp1: str, tcp2: str):
    """Both TCPs trace 10mm circles in XZ plane for 5 seconds."""
    print("\n=== Dual-arm TCP jogging ===")
    duration = 5.0  # seconds
    radius = 5.0  # mm
    frequency = 0.3  # Hz

    async with jog_tcp({mg1: tcp1, mg2: tcp2}) as jogger:
        t0 = time.monotonic()
        start1 = None
        start2 = None
        async for states in jogger:  # yields at ~100Hz
            t = time.monotonic() - t0
            if t >= duration:
                break
            if start1 is None:
                start1 = states[mg1].pose
                start2 = states[mg2].pose
            angle = 2 * math.pi * frequency * t
            jogger.set_target({
                mg1: Pose(
                    start1.position[0] + radius * math.cos(angle),
                    start1.position[1],
                    start1.position[2] + radius * math.sin(angle),
                    *start1.orientation,
                ),
                mg2: Pose(
                    start2.position[0] + radius * math.cos(-angle),
                    start2.position[1],
                    start2.position[2] + radius * math.sin(-angle),
                    *start2.orientation,
                ),
            })


# ---------------------------------------------------------------------------
# Program
# ---------------------------------------------------------------------------


@nova.program(
    id="jogging_dual_arm",
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
async def jogging_dual_arm(ctx: nova.ProgramContext):
    cell = ctx.nova.cell()
    mg1 = (await cell.controller("ur5e-left"))[0]
    mg2 = (await cell.controller("ur5e-right"))[0]
    tcp1 = (await mg1.tcp_names())[0]
    tcp2 = (await mg2.tcp_names())[0]

    try:
        await demo_single_joint(mg1)
        await demo_single_joint_chunked(mg1)
        await demo_single_tcp(mg1, tcp1)
        await demo_dual_joint(mg1, mg2)
        await demo_dual_tcp(mg1, mg2, tcp1, tcp2)
        print("\nAll demos complete.")
    except MotionError as e:
        print(f"\nMotion error (joint limit / collision): {e}")
    except EmergencyStopError as e:
        print(f"\nEmergency stop on controller '{e.controller_id}': {e.safety_state}")


if __name__ == "__main__":
    run_program(jogging_dual_arm)
