"""Single-arm TCP jogging: trace a 50mm circle in XZ plane."""

import math
import time

import nova
from nova import run_program, viewers
from nova.types import Pose
from policy import jog_tcp

START_JOINTS = [1.169, -0.733, 1.745, -3.054, 0.872, 2.094]


@nova.program(
    id="jogging_single_tcp",
    name="Single-Arm TCP Jogging",
    viewer=viewers.Rerun(),
)
async def main(ctx: nova.ProgramContext):
    cell = ctx.nova.cell()
    mg = (await cell.controller("ur5e-left"))[0]
    tcp_name = (await mg.tcp_names())[0]

    duration = 5.0
    radius = 50.0

    async with jog_tcp(mg, tcp=tcp_name, start_joint_position=START_JOINTS) as jogger:
        t0 = time.monotonic()
        start_pose = None
        center_x = 0.0
        center_z = 0.0
        async for state in jogger:
            t = time.monotonic() - t0
            if t >= duration:
                break
            if start_pose is None:
                start_pose = state.pose
                center_x = start_pose.position[0] - radius
                center_z = start_pose.position[2]
            angle = 2 * math.pi * (t / duration)
            jogger.set_target(
                Pose(
                    center_x + radius * math.cos(angle),
                    start_pose.position[1],
                    center_z + radius * math.sin(angle),
                    *start_pose.orientation,
                )
            )


if __name__ == "__main__":
    run_program(main)
