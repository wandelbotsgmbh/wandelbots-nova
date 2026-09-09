"""Two robots sharing one workspace zone, arbitrated by ``nova.interlock``.

Two virtual robots each run a cycle of "work in my own area, then enter the
shared zone, then retreat".  Entering the zone is guarded by an interlock: the
robot takes the zone lock before it plans the motion in, and gives it back only
after it has retreated.  Whoever comes second waits, robot stationary, until the
zone is free again — the software equivalent of the PLC ``Roboterverriegelung``
(``MAKRO 20``) handshake between two KUKA robots.

What to watch in the output:

- ``waiting for … (held by …)`` lines: the second robot blocks on the lock and
  continues as soon as the first one has retreated.
- No zone is ever entered while the other robot holds it, although both cycles
  run concurrently and nothing else synchronizes them.

The rules the example demonstrates (see ``nova.interlock`` for the reasoning):

1. Take the **complete** set of zones a step needs in **one** ``hold``; a second
   acquire while holding raises ``AlreadyHeldError``.  No hold-and-wait means no
   deadlock, even with more robots and more zones.
2. ``hold`` releases only on a **clean** exit of the block.  A program that
   fails inside the zone keeps the lock — the robot may have stopped in there —
   so the partner waits instead of colliding.  There is deliberately no TTL.
3. Recovery is explicit: at program start, with the robot at home and therefore
   known to be clear, ``release_all(include_previous_runs=True)`` drops locks a
   previous crashed run of *this robot* left behind.

Both robots run in one process here for convenience.  In production each robot
is its own process (or pod); nothing changes, because the lock state lives in
the NATS JetStream KV bucket of the cell, not in the process.

This is coordination logic, **not a safety function**.  The safety-rated zone
monitoring of the cell stays responsible for preventing collisions.

Prerequisites:
- Create a NOVA instance
- Set env variables (you can specify them in an .env file):
    - NOVA_API=<api>
    - NOVA_ACCESS_TOKEN=<token>
"""

import asyncio
import logging

import nova
from nova import Controller, api, run_program
from nova.actions import cartesian_ptp, joint_ptp
from nova.cell import virtual_controller
from nova.interlock import InterlockClient, LockId
from nova.types import Pose

ROBOT_A = "interlock-a"
ROBOT_B = "interlock-b"

# The zone shared by the two robots.  A lock is a *pair* relationship — "the
# space A and B both use" — identified by the two robot names and a slot number
# (VASS numbers the zones between one robot pair 1..16).  There is no geometry
# attached; which motions need the zone is the programmer's knowledge.
SHARED_ZONE = LockId.of(ROBOT_A, ROBOT_B, 1)

CYCLES = 3

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s %(levelname)-7s %(message)s"
)
log = logging.getLogger("interlock_example")


async def robot_cycle(controller: Controller, locks: InterlockClient) -> None:
    """One robot's program: own work, then the shared zone, repeated."""
    motion_group = controller[0]
    tcp = (await motion_group.tcp_names())[0]
    home = await motion_group.joints()
    start_pose = await motion_group.tcp_pose(tcp)

    # Two small excursions from the start pose.  Where they point is irrelevant
    # here; the point is which one is guarded by the interlock.
    own_area = start_pose @ Pose((0, 0, -80, 0, 0, 0))
    shared_zone = start_pose @ Pose((120, 0, 0, 0, 0, 0))

    # Rule 3 — the robot is at home, i.e. known to be outside every zone, so any
    # lock a previous run of this robot left behind (it crashed, was killed …)
    # is safe to drop now.  Anywhere else this would be the wrong call.
    stale = await locks.release_all(include_previous_runs=True)
    if stale:
        log.warning(
            "[%s] recovered stale locks from a previous run: %s", locks.robot, stale
        )

    for cycle in range(1, CYCLES + 1):
        # Work that does not touch the shared zone needs no lock.
        await motion_group.plan_and_execute(
            [joint_ptp(home), cartesian_ptp(own_area)], tcp=tcp
        )

        # Rule 1 — everything this step needs, in one call, robot stationary.
        # `acquire` blocks until the zone is free; only then is motion planned.
        log.info("[%s] cycle %d: requesting %s", locks.robot, cycle, SHARED_ZONE.key)
        async with locks.hold([SHARED_ZONE], label=f"cycle {cycle}") as grant:
            log.info("[%s] cycle %d: entering the shared zone", locks.robot, cycle)
            await motion_group.plan_and_execute([cartesian_ptp(shared_zone)], tcp=tcp)
            # … the actual work in the zone happens here …
            await motion_group.plan_and_execute([joint_ptp(home)], tcp=tcp)
            log.info(
                "[%s] cycle %d: retreated, releasing %s", locks.robot, cycle, grant.keys
            )
        # Rule 2 — the clean exit above is the release.  Had anything raised
        # inside the block, the lock would still be held now.

    log.info("[%s] done, holding %s", locks.robot, locks.held or "nothing")


@nova.program(
    id="interlock_two_robots",
    name="Interlock: two robots, one shared zone",
    viewer=nova.viewers.Rerun(),
    preconditions=nova.ProgramPreconditions(
        controllers=[
            virtual_controller(
                name=ROBOT_A,
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type="universalrobots-ur10e",
            ),
            virtual_controller(
                name=ROBOT_B,
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type="universalrobots-ur5e",
            ),
        ],
        cleanup_controllers=False,
    ),
)
async def interlock_two_robots(ctx: nova.ProgramContext):
    cell = ctx.nova.cell()
    controller_a = await cell.controller(ROBOT_A)
    controller_b = await cell.controller(ROBOT_B)

    # One client per robot identity, all on the cell's NATS connection.  The
    # bucket `nova_cells_<cell>_interlocks` is created on first use.
    locks_a = InterlockClient(ctx.nova.nats, cell=cell.cell_id, robot=ROBOT_A)
    locks_b = InterlockClient(ctx.nova.nats, cell=cell.cell_id, robot=ROBOT_B)

    # Both cycles start at the same moment and race for the zone; the interlock
    # is the only thing serializing them.
    await asyncio.gather(
        robot_cycle(controller_a, locks_a), robot_cycle(controller_b, locks_b)
    )

    # Operator's view of the bucket — empty after two clean runs.
    remaining = await locks_a.inspect()
    log.info("locks still held in the cell: %s", remaining or "none")


if __name__ == "__main__":
    run_program(interlock_two_robots)
