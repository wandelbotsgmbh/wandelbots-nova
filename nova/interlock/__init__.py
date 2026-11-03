"""Robot↔robot interlocks backed by NATS JetStream KV.

Prototype replacement for a PLC-based robot interlock handshake.

.. warning::
    **Experimental and not a safety function.**  This coordinates robots; it does
    not guarantee they cannot collide.  The cell's safety-rated zone monitoring
    remains responsible for that.  The API will change without notice.

Typical use from a robot program::

    from nova.interlock import LockId, RobotOwnership

    # one live process per robot id: refuses to start next to another live one
    async with RobotOwnership(ctx.nova.nats, cell="cell", robot="robot-a") as owner:
        locks = owner.client()
        await locks.release_all(include_previous_runs=True)  # only with the robot at home

        # take every slot the step needs, atomically, robot stationary
        async with locks.hold(
            [LockId.of("robot-a", "robot-b", 9), LockId.of("robot-a", "robot-c", 11)],
            label="pick from the shared table",
        ):
            await plan_and_execute_with_cursor(motion_group, actions, tcp=tcp)
        # a *clean* exit of the block is the release of the zones;
        # on an exception or cancellation the locks stay held — the robot may have
        # stopped inside the zone.  Recover with release_all() once confirmed clear.

A step that a fault handler retries passes ``request_id`` to ``acquire``: a retry
after the grant was already taken gets the same grant back instead of
``AlreadyHeldError``.
"""

from nova.interlock.client import CORRUPT_HOLDER, InterlockClient
from nova.interlock.models import (
    BUCKET_TEMPLATE,
    OWNERS_BUCKET_TEMPLATE,
    AlreadyHeldError,
    ForeignRelease,
    Grant,
    GrantLost,
    InterlockError,
    InterlockTimeout,
    LockId,
    LockRecord,
    OwnershipConflict,
    OwnershipLost,
    OwnershipRequired,
)
from nova.interlock.ownership import OwnerRecord, RobotOwnership

__all__ = [
    "BUCKET_TEMPLATE",
    "CORRUPT_HOLDER",
    "OWNERS_BUCKET_TEMPLATE",
    "AlreadyHeldError",
    "ForeignRelease",
    "Grant",
    "GrantLost",
    "InterlockClient",
    "InterlockError",
    "InterlockTimeout",
    "LockId",
    "LockRecord",
    "OwnerRecord",
    "OwnershipConflict",
    "OwnershipLost",
    "OwnershipRequired",
    "RobotOwnership",
]
