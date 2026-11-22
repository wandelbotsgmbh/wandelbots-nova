"""Single-arm joint jogging: oscillate joint 0 for 5 seconds."""

import math

import nova
from novapolicy import LiveBuffer, jog_joints

HOME = [1.169, -0.733, 1.745, -3.054, 0.872, 2.094]


@nova.program(
    id="jogging_single_joint",
    name="Single-Arm Joint Jogging",
    viewer=nova.viewers.Rerun(),
)
async def main(ctx: nova.ProgramContext):
    cell = ctx.nova.cell()
    mg = (await cell.controller("ur5e-left"))[0]

    duration = 5.0
    amplitude = 0.3
    frequency = 0.5

    # set_target sends live targets, which go through the default 500 ms
    # LiveBuffer: recent targets are replayed as a waypoint horizon, so the robot tracks
    # the moving target smoothly, about 500 ms behind. Without it each target goes out
    # alone and the robot brakes toward every one of them.
    async with jog_joints(
        mg, start_joint_position=HOME, live_buffer=LiveBuffer(window_ms=500)
    ) as jogger:
        async for _ in jogger:
            t = jogger.elapsed
            if t >= duration:
                break
            target = list(HOME)
            target[0] += amplitude * math.sin(2 * math.pi * frequency * t)
            jogger.set_target(target)


if __name__ == "__main__":
    nova.run_program(main)
