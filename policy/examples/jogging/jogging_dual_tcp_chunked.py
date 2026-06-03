"""Dual-arm TCP jogging (chunked): both TCPs trace 50mm circles in XZ plane.

Uses 100-step chunks at 10ms for smooth motion with 1s lookahead.
"""

import math
import time

import nova
from nova import run_program, viewers
from policy import jog_tcp

HOME_LEFT = [1.169, -0.733, 1.745, -3.054, 0.872, 2.094]
HOME_RIGHT = [-1.169, -2.3911, -1.8675, 0.0, -0.872, -2.094]


@nova.program(
    id="jogging_dual_tcp_chunked",
    name="Dual-Arm TCP Jogging (Chunked)",
    viewer=viewers.Rerun(),
)
async def main(ctx: nova.ProgramContext):
    cell = ctx.nova.cell()
    mg1 = (await cell.controller("ur5e-left"))[0]
    mg2 = (await cell.controller("ur5e-right"))[0]
    tcp1 = (await mg1.tcp_names())[0]
    tcp2 = (await mg2.tcp_names())[0]

    duration = 10.0
    radius = 50.0
    frequency = 0.3
    chunk_size = 100
    dt_ms = 10.0
    dt_s = dt_ms / 1000.0

    async with jog_tcp(
        {mg1: tcp1, mg2: tcp2},
        start_joint_position={mg1: HOME_LEFT, mg2: HOME_RIGHT},
    ) as jogger:
        t0 = time.monotonic()
        start1 = None
        start2 = None
        center1_x = 0.0
        center1_z = 0.0
        center2_x = 0.0
        center2_z = 0.0
        async for states in jogger:
            t = time.monotonic() - t0
            if t >= duration:
                break
            if start1 is None:
                start1 = states[mg1].pose
                start2 = states[mg2].pose
                center1_x = start1.position[0] - radius
                center1_z = start1.position[2]
                center2_x = start2.position[0] - radius
                center2_z = start2.position[2]
            chunk1 = []
            chunk2 = []
            for i in range(chunk_size):
                future_t = t + i * dt_s
                angle = 2 * math.pi * frequency * future_t
                chunk1.append([
                    center1_x + radius * math.cos(angle),
                    start1.position[1],
                    center1_z + radius * math.sin(angle),
                    *start1.orientation,
                ])
                chunk2.append([
                    center2_x + radius * math.cos(-angle),
                    start2.position[1],
                    center2_z + radius * math.sin(-angle),
                    *start2.orientation,
                ])
            jogger.set_target({mg1: chunk1, mg2: chunk2}, dt_ms=dt_ms)


if __name__ == "__main__":
    run_program(main)
