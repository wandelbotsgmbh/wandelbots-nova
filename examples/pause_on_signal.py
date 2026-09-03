"""
Example: pause robot motion on an IO signal, one signal per motion group.

``execute()`` / ``plan_and_execute()`` accept ``pause_on_io``: an IO condition the robot
controller watches while it runs the trajectory. While the condition holds, the controller
pauses the robot gracefully on path; the SDK keeps the execution open, waits for the
signal to clear and resumes it, so the ``await`` returns only when the trajectory is
traversed (see docs/architecture/adr/002-io-pause-is-resumable.md).

The signal is per motion group: give every robot its own signal to pause them
individually, or the same signal to several robots to pause them together. Both
controller IOs (``IOOrigin.CONTROLLER``) and bus IOs such as Profinet variables
(``IOOrigin.BUS_IO``) work.

This example runs two virtual robots (KUKA + UR), each armed with its own controller
output as pause signal, and toggles the signals from a second task while they move:
first only the KUKA pauses, then both pause, then everything finishes.

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
from nova.cell import virtual_controller
from nova.program import ProgramPreconditions
from nova.types import MotionSettings

KUKA = "kuka-pause-example"
UR = "ur-pause-example"
# One pause signal per robot — here a digital output of each controller; a Profinet
# variable with ``IOOrigin.BUS_IO`` works the same way.
PAUSE_IO = {KUKA: "OUT#900", UR: "digital_out[0]"}


def pause_when_high(io: str) -> api.models.PauseOnIO:
    """Pause while ``io`` reads ``True``; resume when it reads ``False``."""
    return api.models.PauseOnIO(
        io=api.models.IOBooleanValue(io=io, value=True),
        comparator=api.models.Comparator.COMPARATOR_EQUALS,
        io_origin=api.models.IOOrigin.CONTROLLER,
    )


async def operator(kuka, ur) -> None:
    """Stand in for the PLC / the person responsible for the cell."""
    await asyncio.sleep(3.0)
    print("operator: pausing the KUKA only")
    await kuka.write(PAUSE_IO[KUKA], True)
    await asyncio.sleep(3.0)
    print("operator: pausing the UR too")
    await ur.write(PAUSE_IO[UR], True)
    await asyncio.sleep(3.0)
    print("operator: releasing both")
    await kuka.write(PAUSE_IO[KUKA], False)
    await ur.write(PAUSE_IO[UR], False)


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
        cleanup_controllers=True,
    ),
)
async def main(ctx: nova.ProgramContext):
    cell = ctx.cell
    kuka = await cell.controller(KUKA)
    ur = await cell.controller(UR)
    await kuka.write(PAUSE_IO[KUKA], False)
    await ur.write(PAUSE_IO[UR], False)

    async with kuka[0] as kuka_mg, ur[0] as ur_mg:
        slow = MotionSettings(tcp_velocity_limit=30)

        async def move(mg, controller_name: str):
            joints = list(await mg.joints())
            joints[0] += 1.0
            await mg.plan_and_execute(
                [jnt(joints, settings=slow)],
                tcp="Flange",
                pause_on_io=pause_when_high(PAUSE_IO[controller_name]),
            )
            print(f"{controller_name}: trajectory finished")

        await asyncio.gather(move(kuka_mg, KUKA), move(ur_mg, UR), operator(kuka, ur))


if __name__ == "__main__":
    run_program(main)
