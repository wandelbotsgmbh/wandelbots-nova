"""Dual-arm joint jogging (chunked): mirror-symmetric oscillation for 5 seconds.

Uses 8-step chunks at 33ms for smooth motion with 264ms lookahead.
"""

import math

import nova
from novapolicy import jog_joints

HOME_LEFT = [1.169, -0.733, 1.745, -3.054, 0.872, 2.094]
HOME_RIGHT = [-1.169, -2.3911, -1.8675, 0.0, -0.872, -2.094]


@nova.program(
    id="jogging_dual_joint_chunked",
    name="Dual-Arm Joint Jogging (Chunked)",
    viewer=nova.viewers.Rerun(),
)
async def main(ctx: nova.ProgramContext):
    cell = ctx.nova.cell()
    mg1 = (await cell.controller("ur5e-left"))[0]
    mg2 = (await cell.controller("ur5e-right"))[0]

    duration = 5.0
    amplitude = 0.3
    frequency = 0.5
    chunk_size = 8
    dt_ms = 33.0
    dt_s = dt_ms / 1000.0

    async with jog_joints(
        [mg1, mg2], start_joint_position={mg1: HOME_LEFT, mg2: HOME_RIGHT}
    ) as jogger:
        async for _ in jogger:
            t = jogger.elapsed
            if t >= duration:
                break
            chunk1 = []
            chunk2 = []
            for i in range(chunk_size):
                wave = amplitude * math.sin(2 * math.pi * frequency * (t + i * dt_s))
                step1 = list(HOME_LEFT)
                step2 = list(HOME_RIGHT)
                step1[0] += wave
                step2[0] -= wave
                chunk1.append(step1)
                chunk2.append(step2)
            jogger.set_target({mg1: chunk1, mg2: chunk2}, dt_ms=dt_ms)


if __name__ == "__main__":
    nova.run_program(main)
