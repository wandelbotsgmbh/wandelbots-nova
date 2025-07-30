import nats
from decouple import config

from nova.core.logging import logger

from . import BaseCycleEvent, cycle_failed, cycle_finished, cycle_started

NATS_SERVERS = config("NATS_SERVERS", default=None)
if not NATS_SERVERS:
    NATS_SERVERS = config("NATS_BROKER", default="nats://nats.wandelbots.svc:4222", cast=str)

NATS_SUBJECT_CYCLE = config("NATS_SUBJECT_CYCLE", default="cell.process.cycle")

_nats_client: nats.NATS | None = None


async def get_client() -> nats.NATS:
    global _nats_client
    if _nats_client is not None:
        return _nats_client
    _nats_client = await nats.connect(servers=NATS_SERVERS)
    return _nats_client


async def close():
    global _nats_client
    if _nats_client:
        logger.debug("Closing NATS client")
        await _nats_client.drain()
        _nats_client = None


async def cycle_event_handler(sender, message: BaseCycleEvent, **kwargs):
    event_type = message.event_type
    logger.debug(f"NATS event handler received {event_type} event")
    nats_client = await get_client()
    try:
        await nats_client.publish(NATS_SUBJECT_CYCLE, message.model_dump_json().encode())
        logger.debug(f"Published {event_type} event to NATS")
    except Exception as e:
        logger.error(f"Failed to publish {event_type} event to NATS: {e}")


if NATS_SERVERS:
    for signal in [cycle_started, cycle_finished, cycle_failed]:
        event_type = signal.name
        logger.debug(f"Connecting NATS event handler for {event_type} event")
        signal.connect(cycle_event_handler)
