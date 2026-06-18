"""Dual-arm joint jogging: mirror-symmetric oscillation for 5 seconds."""

import math

import nova
from novapolicy import jog_joints

HOME_LEFT = [1.169, -0.733, 1.745, -3.054, 0.872, 2.094]
HOME_RIGHT = [-1.169, -2.3911, -1.8675, 0.0, -0.872, -2.094]


@nova.program(
    id="jogging_dual_joint",
    name="Dual-Arm Joint Jogging",
    viewer=nova.viewers.Rerun(),
)
async def main(ctx: nova.ProgramContext):
    cell = ctx.nova.cell()
    mg1 = (await cell.controller("ur5e-left"))[0]
    mg2 = (await cell.controller("ur5e-right"))[0]

    duration = 5.0
    amplitude = 0.3
    frequency = 0.5

    async with jog_joints(
        [mg1, mg2], start_joint_position={mg1: HOME_LEFT, mg2: HOME_RIGHT}
    ) as jogger:
        async for _ in jogger:
            t = jogger.elapsed
            if t >= duration:
                break
            wave = amplitude * math.sin(2 * math.pi * frequency * t)
            t1 = list(HOME_LEFT)
            t2 = list(HOME_RIGHT)
            t1[0] += wave
            t2[0] -= wave
            jogger.set_target({mg1: t1, mg2: t2})


if __name__ == "__main__":
    nova.run_program(main)
