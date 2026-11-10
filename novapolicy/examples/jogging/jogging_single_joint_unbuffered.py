"""Single-arm joint jogging with lone live targets: no live buffer, no delay_until_braking.

Oscillates joint 0 of a virtual UR10e for 6 seconds. Every target goes out on its
own as soon as it is set, so the robot follows about 100 ms behind instead of the
buffer's 500 ms.

    NOVA_API=http://172.31.13.175 PYTHONPATH=. uv run python \
        novapolicy/examples/jogging/jogging_single_joint_unbuffered.py
"""

import math

import nova
from nova.cell import virtual_controller
from novapolicy import jog_joints

CONTROLLER = "ur10e-unbuffered"
HOME = [1.169, -0.733, 1.745, -3.054, 0.872, 2.094]


@nova.program(
    id="jogging_single_joint_unbuffered",
    name="Single-Arm Joint Jogging, Unbuffered",
    viewer=nova.viewers.Rerun(),
    preconditions=nova.ProgramPreconditions(
        controllers=[
            virtual_controller(
                name=CONTROLLER,
                manufacturer=nova.api.models.Manufacturer.UNIVERSALROBOTS,
                type="universalrobots-ur10e",
            )
        ],
        cleanup_controllers=False,
    ),
)
async def main(ctx: nova.ProgramContext):
    mg = (await ctx.nova.cell().controller(CONTROLLER))[0]

    duration = 6.0
    amplitude = 0.3
    frequency = 0.5

    async with jog_joints(mg, start_joint_position=HOME, buffer_window_ms=0) as jogger:
        async for _ in jogger:
            t = jogger.elapsed
            if t >= duration:
                break
            target = list(HOME)
            target[0] += amplitude * math.sin(2 * math.pi * frequency * t)
            jogger.set_target(target)


if __name__ == "__main__":
    nova.run_program(main)
