"""Two robots sharing one workspace zone, arbitrated by ``nova.interlock``.

Two virtual KUKA robots stand 2 m apart on the X axis, both facing +Y, so at
rest their envelopes do not overlap.  Each cycle a robot brings its flange to
the meeting point midway between the two bases, (0, 0, 1500) mm, flange down —
a point both robots can reach, so that is the shared zone.  It gets there in
two stages: first it turns its base joint 45° toward the partner, to the
*approach* point, which is still clear of the zone and needs no lock; there,
stationary, it takes the zone lock, moves to the meeting point, comes back to
the approach point and only then gives the lock back.  Whoever comes second
waits at its approach point until the zone is free again — the software
equivalent of the PLC ``Roboterverriegelung`` (``MAKRO 20``) handshake between
two KUKA robots.

What to watch (in the log, and in rerun where the two robots are drawn at their
mountings):

- ``waiting for … (held by …)`` lines: the second robot blocks on the lock and
  continues as soon as the first one has retreated.
- The two flanges visit the meeting point alternately, never together,
  although both cycles run concurrently and nothing else synchronizes them.

The rules the example demonstrates (see ``nova.interlock`` for the reasoning):

1. Take the **complete** set of zones a step needs in **one** ``hold``; a second
   acquire while holding raises ``AlreadyHeldError``.  No hold-and-wait means no
   deadlock, even with more robots and more zones.
2. ``hold`` releases only on a **clean** exit of the block.  A program that
   fails inside the zone keeps the lock — the robot may have stopped in there —
   so the partner waits instead of colliding.  There is deliberately no TTL.
3. Recovery is explicit: at the start, with the arm turned away from the
   partner and therefore known to be clear,
   ``release_all(include_previous_runs=True)`` drops locks a previous crashed
   run of *this robot* left behind.

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
import math

import nova
from nova import Controller, api, run_program
from nova.actions import cartesian_ptp, joint_ptp
from nova.cell import virtual_controller
from nova.interlock import InterlockClient, LockId
from nova.types import MotionSettings, Pose

ROBOT_A = "interlock-a"
ROBOT_B = "interlock-b"

# Where each robot stands (world frame, mm) and which way it faces: A at
# x = -1000, B at x = +1000, both turned by +90° so their arms point along +Y —
# away from each other.
FACING = math.pi / 2
PLACEMENT = {ROBOT_A: (-1000.0, FACING), ROBOT_B: (1000.0, FACING)}

# The approach point: base joint (A1) turned 45° toward the partner.  A must
# turn from +Y toward +X, B from +Y toward -X.  KUKA's A1 counts *clockwise*
# seen from above (a positive value is a negative rotation about the base Z
# axis), so A turns positive and B negative — checked at runtime by
# _check_turn_direction, because this is the one robot-model convention the
# example relies on.
TOWARD_PARTNER = {ROBOT_A: 1.0, ROBOT_B: -1.0}
APPROACH_ANGLE = math.radians(45)

# The meeting point, world frame, mm: midway between the bases, 1.5 m up.  Its
# orientation is taken from the rest pose at runtime (flange pointing down).
MEETING_POINT = (0.0, 0.0, 1500.0)

# The zone shared by the two robots.  A lock is a *pair* relationship — "the
# space A and B both use" — identified by the two robot names and a slot number
# (VASS numbers the zones between one robot pair 1..16).  There is no geometry
# attached; which motions need the zone is the programmer's knowledge.
SHARED_ZONE = LockId.of(ROBOT_A, ROBOT_B, 1)

CYCLES = 10

# TCP velocity limit for every motion, in mm/s.
SPEED = MotionSettings(tcp_velocity_limit=1000)

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)-7s %(message)s")
log = logging.getLogger("interlock_example")


def _values(vector) -> list[float]:
    """Read an API vector back as floats (tuple alias or RootModel, either shape)."""
    root = getattr(vector, "root", None)
    return [float(v) for v in (root if root is not None else vector)]


async def place_robot(ctx: nova.ProgramContext, cell_id: str, name: str) -> None:
    """Mount a virtual robot at its PLACEMENT, only if it is not already there.

    Writing a mounting makes the virtual robot re-initialize (~20 s), and while
    that runs the description can report an empty TCP map and planning fails —
    so skip the write when the controller already carries the placement, and
    otherwise wait until it reports the new mounting *and* its TCPs again.
    """
    x, yaw = PLACEMENT[name]
    target = [x, 0.0, 0.0, 0.0, 0.0, yaw]  # position + rotation vector (yaw about Z)
    motion_group = f"0@{name}"
    description_api = ctx.nova.api.motion_group_api

    def is_placed(description) -> bool:
        mounting = getattr(description, "mounting", None)
        if mounting is None:
            return False
        current = _values(mounting.position) + _values(mounting.orientation)
        return all(abs(c - t) < 1e-3 for c, t in zip(current, target)) and bool(description.tcps)

    description = await description_api.get_motion_group_description(
        cell=cell_id, controller=name, motion_group=motion_group
    )
    if is_placed(description):
        log.info("[%s] already mounted at x=%+.0f mm", name, x)
        return

    log.info(
        "[%s] mounting at x=%+.0f mm, yaw %.0f° — the robot re-initializes",
        name,
        x,
        math.degrees(yaw),
    )
    await ctx.nova.api.virtual_robot_setup_api.set_virtual_controller_mounting(
        cell=cell_id,
        controller=name,
        motion_group=motion_group,
        coordinate_system=api.models.CoordinateSystem(
            coordinate_system="world",
            name="mounting",
            position=(x, 0.0, 0.0),
            orientation=[0.0, 0.0, yaw],
            orientation_type=api.models.OrientationType.EULER_ANGLES_EXTRINSIC_XYZ,
        ),
    )
    deadline = asyncio.get_running_loop().time() + 120
    while asyncio.get_running_loop().time() < deadline:
        await asyncio.sleep(2)
        try:
            description = await description_api.get_motion_group_description(
                cell=cell_id, controller=name, motion_group=motion_group
            )
        except Exception:
            continue  # not reachable yet — that is the re-initialization
        if is_placed(description):
            log.info("[%s] back with the new mounting", name)
            return
    raise TimeoutError(f"{name} did not come back after the mounting change")


async def robot_cycle(controller: Controller, locks: InterlockClient) -> None:
    """One robot's program: approach, visit the meeting point, retreat, repeated."""
    motion_group = controller[0]
    tcp = (await motion_group.tcp_names())[0]
    # Rest = base joint at 0, arm along the base X axis, i.e. along world +Y,
    # away from the partner.  Defined, not measured: the robot may still be
    # turned from an earlier run, and "wherever I am now" is not a rest pose.
    rest = list(await motion_group.joints())
    rest[0] = 0.0
    direction = TOWARD_PARTNER[locks.robot]

    def turned_by(angle: float) -> list[float]:
        joints = list(rest)
        joints[0] = rest[0] + direction * angle
        return joints

    approach = turned_by(APPROACH_ANGLE)  # 45° toward the partner, still clear of the zone

    await motion_group.plan_and_execute([joint_ptp(rest, settings=SPEED)], tcp=tcp)

    # The meeting point keeps the rest pose's orientation — flange pointing down.
    orientation = (await motion_group.tcp_pose(tcp)).to_tuple()[3:]
    meeting_point = Pose((*MEETING_POINT, *orientation))

    # Rule 3 — at rest the arm points away from the partner, i.e. is known to
    # be outside the zone, so any lock a previous run of this robot left behind
    # (it crashed, was killed …) is safe to drop now.  Anywhere else this would
    # be wrong.
    stale = await locks.release_all(include_previous_runs=True)
    if stale:
        log.warning("[%s] recovered stale locks from a previous run: %s", locks.robot, stale)

    for cycle in range(1, CYCLES + 1):
        # The approach point does not touch the shared zone and needs no lock.
        await motion_group.plan_and_execute([joint_ptp(approach, settings=SPEED)], tcp=tcp)
        if cycle == 1:
            await _check_turn_direction(motion_group, tcp, locks.robot)

        # Rule 1 — everything this step needs, in one call, robot stationary at
        # the approach point.  `acquire` blocks until the zone is free; only
        # then is the motion to the meeting point planned.
        log.info("[%s] cycle %d: at approach, requesting %s", locks.robot, cycle, SHARED_ZONE.key)
        async with locks.hold([SHARED_ZONE], label=f"cycle {cycle}") as grant:
            log.info("[%s] cycle %d: moving to the meeting point", locks.robot, cycle)
            await motion_group.plan_and_execute(
                [cartesian_ptp(meeting_point, settings=SPEED)], tcp=tcp
            )
            # … the actual work at the meeting point happens here …
            await motion_group.plan_and_execute([joint_ptp(approach, settings=SPEED)], tcp=tcp)
            log.info(
                "[%s] cycle %d: back at approach, releasing %s", locks.robot, cycle, grant.keys
            )
        # Rule 2 — the clean exit above is the release.  Had anything raised
        # inside the block, the lock would still be held now.

        await motion_group.plan_and_execute([joint_ptp(rest, settings=SPEED)], tcp=tcp)

    log.info("[%s] done, holding %s", locks.robot, locks.held or "nothing")


async def _check_turn_direction(motion_group, tcp: str, robot: str) -> None:
    """Fail loudly if the base joint turned the arm away from the partner.

    The sign convention of joint 1 is the one thing this example assumes about
    the robot model; a wrong sign would make both arms swing outward and the
    interlock would guard empty space.  At 45° the TCP must be closer to the
    cell middle (x = 0) than the robot's own base.
    """
    base_x, _ = PLACEMENT[robot]
    x = (await motion_group.tcp_pose(tcp)).to_tuple()[0]
    if abs(x) >= abs(base_x):
        raise RuntimeError(
            f"{robot}: after turning 45° the TCP is at x={x:.0f} mm, not between the "
            f"bases (base at x={base_x:.0f}) — the base joint turned the wrong way "
            f"or not at all; check TOWARD_PARTNER for this robot model"
        )


@nova.program(
    id="interlock_two_robots",
    name="Interlock: two robots, one shared zone",
    # Wall-clock timeline: by default the viewer packs each robot's trajectories
    # back to back and drops the time spent waiting for the lock, so the replay
    # would show both robots at the meeting point at once although they took
    # turns.  With wall_clock=True the replay follows what actually happened.
    viewer=nova.viewers.Rerun(wall_clock=True),
    preconditions=nova.ProgramPreconditions(
        controllers=[
            virtual_controller(
                name=ROBOT_A, manufacturer=api.models.Manufacturer.KUKA, type="kuka-kr240_r2900"
            ),
            virtual_controller(
                name=ROBOT_B, manufacturer=api.models.Manufacturer.KUKA, type="kuka-kr270_r2700"
            ),
        ],
        cleanup_controllers=False,
    ),
)
async def interlock_two_robots(ctx: nova.ProgramContext):
    cell = ctx.nova.cell()
    controller_a = await cell.controller(ROBOT_A)
    controller_b = await cell.controller(ROBOT_B)

    # Put the robots where they stand in the cell (idempotent, see place_robot).
    await asyncio.gather(
        place_robot(ctx, cell.cell_id, ROBOT_A), place_robot(ctx, cell.cell_id, ROBOT_B)
    )

    # One client per robot identity, all on the cell's NATS connection.  The
    # bucket `nova_cells_<cell>_interlocks` is created on first use.
    locks_a = InterlockClient(ctx.nova.nats, cell=cell.cell_id, robot=ROBOT_A)
    locks_b = InterlockClient(ctx.nova.nats, cell=cell.cell_id, robot=ROBOT_B)

    # Both cycles start at the same moment and race for the zone; the interlock
    # is the only thing serializing them.
    await asyncio.gather(robot_cycle(controller_a, locks_a), robot_cycle(controller_b, locks_b))

    # Operator's view of the bucket — empty after two clean runs.
    remaining = await locks_a.inspect()
    log.info("locks still held in the cell: %s", remaining or "none")


if __name__ == "__main__":
    run_program(interlock_two_robots)
