"""Single-arm joint jogging on the virtual ur10e-clone: oscillate joint 0 for 5s.

Mirrors jogging_single_joint.py but targets the virtual ur10e-clone and the
robot's current home pose, so it can be dry-run before moving the real ur10e.

Run:
    NOVA_API=http://wandelbox-hhmnwy PYTHONPATH=. python \
        policy/examples/jogging/jogging_single_joint_clone.py
"""

import math
import time

import nova
from nova import api, run_program, viewers
from nova.actions import jnt
from nova.types import MotionSettings
from policy import jog_joints

# Current pose of the real ur10e / clone (read from get_state on 2026-06-11).
HOME = [0.3202, -1.8691, 1.9472, -1.6528, -1.5776, 1.8531]

CONTROLLER = "ur10e"  # the real robot (use "ur10e-clone" for the virtual dry-run)


@nova.program(
    id="jogging_single_joint_clone",
    name="Single-Arm Joint Jogging (clone)",
    viewer=viewers.Rerun(),
)
async def main(ctx: nova.ProgramContext):
    cell = ctx.nova.cell()
    mg = (await cell.controller(CONTROLLER))[0]
    tcp = (await mg.tcp_names())[0]

    duration = 5.0
    # Speed limit: peak joint velocity = amplitude * 2*pi * frequency.
    # 0.2 rad * 2*pi * 0.25 Hz ~= 0.31 rad/s (~18 deg/s) -- conservative.
    amplitude = 0.2
    frequency = 0.25

    # PTP to HOME (the current pose). Collision setups are cleared so the cell's
    # safety planes don't reject planning at this exact pose. A slow TCP velocity
    # limit keeps the approach gentle.
    settings = MotionSettings(tcp_velocity_limit=50.0)  # mm/s
    setup = await mg.get_setup(tcp)
    setup.collision_setups = api.models.CollisionSetups({})
    traj = await mg.plan([jnt(HOME, settings=settings)], tcp, motion_group_setup=setup)
    await mg.execute(traj, tcp, actions=[jnt(HOME, settings=settings)])

    async with jog_joints(mg) as jogger:
        t0 = time.monotonic()
        async for _ in jogger:
            t = time.monotonic() - t0
            if t >= duration:
                break
            target = list(HOME)
            target[0] += amplitude * math.sin(2 * math.pi * frequency * t)
            jogger.set_target(target)


if __name__ == "__main__":
    run_program(main)
