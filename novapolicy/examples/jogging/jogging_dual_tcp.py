"""Dual-arm TCP jogging: both TCPs trace 50mm circles in XZ plane."""

import math

import nova
from nova.types import Pose
from novapolicy import jog_tcp

HOME_LEFT = [1.169, -0.733, 1.745, -3.054, 0.872, 2.094]
HOME_RIGHT = [-1.169, -2.3911, -1.8675, 0.0, -0.872, -2.094]


@nova.program(
    id="jogging_dual_tcp",
    name="Dual-Arm TCP Jogging",
    viewer=nova.viewers.Rerun(),
)
async def main(ctx: nova.ProgramContext):
    cell = ctx.nova.cell()
    mg1 = (await cell.controller("ur5e-left"))[0]
    mg2 = (await cell.controller("ur5e-right"))[0]
    tcp1 = (await mg1.tcp_names())[0]
    tcp2 = (await mg2.tcp_names())[0]

    duration = 5.0
    radius = 50.0
    frequency = 0.3

    async with jog_tcp(
        {mg1: tcp1, mg2: tcp2},
        start_joint_position={mg1: HOME_LEFT, mg2: HOME_RIGHT},
    ) as jogger:
        start1 = None
        start2 = None
        async for states in jogger:
            t = jogger.elapsed
            if t >= duration:
                break
            if start1 is None:
                start1 = states[mg1].pose
                start2 = states[mg2].pose
                center1_x = start1.position[0] - radius
                center1_z = start1.position[2]
                center2_x = start2.position[0] - radius
                center2_z = start2.position[2]
            angle = 2 * math.pi * frequency * t
            jogger.set_target(
                {
                    mg1: Pose(
                        center1_x + radius * math.cos(angle),
                        start1.position[1],
                        center1_z + radius * math.sin(angle),
                        *start1.orientation,
                    ),
                    mg2: Pose(
                        center2_x + radius * math.cos(-angle),
                        start2.position[1],
                        center2_z + radius * math.sin(-angle),
                        *start2.orientation,
                    ),
                }
            )


if __name__ == "__main__":
    nova.run_program(main)
