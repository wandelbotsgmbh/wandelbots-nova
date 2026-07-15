"""Single-arm TCP jogging: trace a 50mm circle in XZ plane (chunked, smooth).

Uses 100-step chunks at 10ms for smooth motion with 1s lookahead.
"""

import math

import nova
from novapolicy import jog_tcp

START_JOINTS = [1.169, -0.733, 1.745, -3.054, 0.872, 2.094]


@nova.program(
    id="jogging_single_tcp_chunked",
    name="Single-Arm TCP Jogging (Chunked)",
    viewer=nova.viewers.Rerun(),
)
async def main(ctx: nova.ProgramContext):
    cell = ctx.nova.cell()
    mg = (await cell.controller("ur5e-left"))[0]
    tcp_name = (await mg.tcp_names())[0]

    duration = 10.0
    radius = 50.0
    chunk_size = 100
    dt_ms = 10.0
    dt_s = dt_ms / 1000.0

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
            chunk = []
            for i in range(chunk_size):
                future_t = t + i * dt_s
                angle = 2 * math.pi * (future_t / duration)
                chunk.append([
                    center_x + radius * math.cos(angle),
                    start_pose.position[1],
                    center_z + radius * math.sin(angle),
                    *start_pose.orientation,
                ])
            jogger.set_target(chunk, dt_ms=dt_ms)


if __name__ == "__main__":
    nova.run_program(main)
