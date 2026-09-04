"""Single-arm TCP jogging: trace a 50mm circle in XZ plane.

Starts and stops at rest by easing the *phase* of the circle rather than its
amplitude. Enveloping the amplitude — the right move for an oscillation about a
home pose, as jogging_single_joint_chunked.py does — would drag the TCP to the
circle's centre and back out again. Warping the phase keeps it exactly on the
circle and simply slows it at both ends.
"""

import math

import nova
from nova.types import Pose
from novapolicy import jog_tcp

START_JOINTS = [0.9484, -2.128, 2.2734, -1.7487, -1.5773, 2.552]


@nova.program(id="jogging_single_tcp", name="Single-Arm TCP Jogging", viewer=nova.viewers.Rerun())
async def main(ctx: nova.ProgramContext):
    cell = ctx.nova.cell()
    mg = (await cell.controller("ur10e"))[0]
    tcp_name = (await mg.tcp_names())[0]

    duration = 5.0
    radius = 50.0

    def smootherstep(x: float) -> float:
        """Monotone 0->1 warp with zero velocity *and* zero acceleration at both ends.

        Smoothstep (3x^2 - 2x^3) only zeroes the velocity; its acceleration still
        steps at the ends, which the robot executes as a stumble. This zeroes both,
        so only jerk steps.
        """
        x = min(1.0, max(0.0, x))
        return x**3 * (x * (x * 6 - 15) + 10)

    # set_target sends live poses, which go through the default 500 ms
    # buffer_window_ms: recent poses are replayed as a waypoint horizon, so the TCP
    # tracks that far behind the circle. Pass buffer_window_ms=0 to send each pose
    # alone, at the cost of halting motion between them.
    # No ease_in_s: the phase warp already starts at zero velocity, and the circle
    # starts at the robot's own pose, so there is nothing to blend from.
    async with jog_tcp(mg, tcp=tcp_name, start_joint_position=START_JOINTS) as jogger:
        start_pose = None
        center_x = 0.0
        center_z = 0.0
        async for state in jogger:
            t = jogger.elapsed
            if t >= duration:
                break
            if start_pose is None:
                start_pose = state.pose
                center_x = start_pose.position[0] - radius
                center_z = start_pose.position[2]
            angle = 2 * math.pi * smootherstep(t / duration)
            jogger.set_target(
                Pose(
                    center_x + radius * math.cos(angle),
                    start_pose.position[1],
                    center_z + radius * math.sin(angle),
                    *start_pose.orientation,
                )
            )


if __name__ == "__main__":
    nova.run_program(main)
