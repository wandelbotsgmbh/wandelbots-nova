"""Robot↔robot interlocks backed by NATS JetStream KV.

Prototype replacement for a PLC-based robot interlock handshake.

.. warning::
    **Experimental and not a safety function.**  This coordinates robots; it does
    not guarantee they cannot collide.  The cell's safety-rated zone monitoring
    remains responsible for that.  The API will change without notice.

Typical use from a robot program::

    from nova.interlock import InterlockClient, LockId

    locks = InterlockClient(ctx.nova.nats, cell="cell", robot="robot-a")

    # take every slot the step needs, atomically, robot stationary
    async with locks.hold(
        [LockId.of("robot-a", "robot-b", 9), LockId.of("robot-a", "robot-c", 11)],
        label="pick from the shared table",
    ):
        await plan_and_execute_with_cursor(motion_group, actions, tcp=tcp)
    # a *clean* exit of the block is the release of the zones;
    # on an exception or cancellation the locks stay held — the robot may have
    # stopped inside the zone.  Recover with release_all() once confirmed clear.
"""

from nova.interlock.client import CORRUPT_HOLDER, InterlockClient
from nova.interlock.models import (
    BUCKET_TEMPLATE,
    AlreadyHeldError,
    ForeignRelease,
    Grant,
    InterlockError,
    InterlockTimeout,
    LockId,
    LockRecord,
)

__all__ = [
    "BUCKET_TEMPLATE",
    "CORRUPT_HOLDER",
    "AlreadyHeldError",
    "ForeignRelease",
    "Grant",
    "InterlockClient",
    "InterlockError",
    "InterlockTimeout",
    "LockId",
    "LockRecord",
]
