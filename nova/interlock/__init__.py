"""Robot↔robot interlocks backed by NATS JetStream KV.

Prototype replacement for the PLC ``Roboterverriegelung`` / MAKRO 20 handshake.

.. warning::
    **Experimental and not a safety function.**  This coordinates robots; it does
    not guarantee they cannot collide.  The cell's safety-rated zone monitoring
    remains responsible for that.  The API will change without notice.

Typical use from a robot program::

    from nova.interlock import InterlockClient, LockId

    locks = InterlockClient(ctx.nova.nats, cell="urw1-agr3-sk1", robot="ir340r01")

    # MAKRO 20: take every slot the step needs, atomically, robot stationary
    async with locks.hold(
        [LockId.of("ir340r01", "ir340r02", 9), LockId.of("ir340r01", "ir340r03", 11)],
        label="GEO40 entnehmen",
    ):
        await plan_and_execute_with_cursor(motion_group, actions, tcp=tcp)
    # exiting the block is the "Freigabe Verriegelung" (A49/A51 = EIN)
"""

from nova.interlock.client import InterlockClient
from nova.interlock.models import (
    BUCKET_TEMPLATE,
    ForeignRelease,
    Grant,
    InterlockError,
    InterlockTimeout,
    LockId,
    LockRecord,
)

__all__ = [
    "BUCKET_TEMPLATE",
    "ForeignRelease",
    "Grant",
    "InterlockClient",
    "InterlockError",
    "InterlockTimeout",
    "LockId",
    "LockRecord",
]
