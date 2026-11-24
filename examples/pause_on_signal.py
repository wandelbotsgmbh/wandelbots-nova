"""
Example: a motion-enable signal per robot — the robot moves only while its IO reads True.

``execute()`` / ``plan_and_execute()`` accept ``pause_on_io``: an IO condition the robot
controller watches while it runs the trajectory. While the condition holds, the controller
pauses the robot gracefully on path; the SDK keeps the execution open, waits for the
condition to clear and resumes, so the ``await`` returns only when the trajectory is
traversed (see docs/architecture/adr/002-io-pause-is-resumable.md).

``motion_enable_signal(io)`` builds the fail-safe form of that condition: the robot is
allowed to move while the signal reads True and pauses as soon as it reads False. A PLC
drops the signal to stop the robot — and a broken wire or a lost Profinet connection reads
False as well, so the robot stops in that case too. Resume is automatic once the signal is
back.

The signal is per motion group: give every robot its own signal to stop them individually,
or the same signal to several robots to stop them together. Both controller IOs
(``IOOrigin.CONTROLLER``) and bus IOs such as Profinet variables (``IOOrigin.BUS_IO``) work.

This example runs two virtual robots (KUKA + UR), each enabled by its own controller
output, and drives the signals from a second task while they move: first only the KUKA
loses its enable, then both, then both get it back and finish.

Prerequisites:
- A NOVA instance (see .env / NOVA_API, NOVA_ACCESS_TOKEN)
- Run this example script:
    PYTHONPATH=. uv run python examples/pause_on_signal.py
"""

import asyncio
from math import pi

import nova
from nova import api, run_program
from nova.actions import jnt
from nova.cell import motion_enable_signal, virtual_controller
from nova.program import ProgramPreconditions
from nova.types import MotionSettings

KUKA = "kuka-pause-example"
UR = "ur-pause-example"
# One enable signal per robot — here a digital output of each controller; a Profinet
# variable with ``IOOrigin.BUS_IO`` works the same way.
ENABLE_IO = {KUKA: "OUT#900", UR: "digital_out[0]"}


async def operator(kuka, ur) -> None:
    """Stand in for the PLC / the person responsible for the cell."""
    await asyncio.sleep(3.0)
    print("operator: dropping the KUKA's enable signal — it stops, the UR keeps moving")
    await kuka.write(ENABLE_IO[KUKA], False)
    await asyncio.sleep(3.0)
    print("operator: dropping the UR's enable signal too — both stopped")
    await ur.write(ENABLE_IO[UR], False)
    await asyncio.sleep(3.0)
    print("operator: enable signals back — both resume")
    await kuka.write(ENABLE_IO[KUKA], True)
    await ur.write(ENABLE_IO[UR], True)


@nova.program(
    name="pause_on_signal",
    preconditions=ProgramPreconditions(
        controllers=[
            virtual_controller(
                name=KUKA,
                manufacturer=api.models.Manufacturer.KUKA,
                type="kuka-kr6_r700_sixx",
                position=[0.0, -pi / 2, -pi / 2, 0.0, 0.0, 0.0, 0.0],
            ),
            virtual_controller(
                name=UR,
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type="universalrobots-ur10e",
                position=[pi / 2, -pi / 2, pi / 2, 0.0, pi / 2, 0.0, 0.0],
            ),
        ],
        cleanup_controllers=False,
    ),
)
async def main(ctx: nova.ProgramContext):
    cell = ctx.cell
    kuka = await cell.controller(KUKA)
    ur = await cell.controller(UR)
    # Enable both robots before they start; with the signal low a robot would wait at
    # the start of its trajectory until it is allowed to move.
    await kuka.write(ENABLE_IO[KUKA], True)
    await ur.write(ENABLE_IO[UR], True)

    async with kuka[0] as kuka_mg, ur[0] as ur_mg:
        slow = MotionSettings(tcp_velocity_limit=30)

        async def move(mg, controller_name: str):
            joints = list(await mg.joints())
            joints[0] += 1.0
            await mg.plan_and_execute(
                [jnt(joints, settings=slow)],
                tcp="Flange",
                pause_on_io=motion_enable_signal(
                    ENABLE_IO[controller_name], api.models.IOOrigin.CONTROLLER
                ),
            )
            print(f"{controller_name}: trajectory finished")

        await asyncio.gather(move(kuka_mg, KUKA), move(ur_mg, UR), operator(kuka, ur))


if __name__ == "__main__":
    run_program(main)
