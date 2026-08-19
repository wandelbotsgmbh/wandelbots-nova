"""Single-arm TCP jogging: trace a 50mm circle in XZ plane."""

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

    async with jog_tcp(
        mg, tcp=tcp_name, start_joint_position=START_JOINTS, ease_in_s=0.5
    ) as jogger:
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
    nova.run_program(main)
