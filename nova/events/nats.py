import asyncio
from typing import Optional

import nats
from decouple import config

from nova.core.logging import logger

from . import BaseCycleEvent, ProgramStateChangeEvent, cycle_failed, cycle_finished, cycle_started

NATS_SERVERS = config("NATS_SERVERS", default=None)
if not NATS_SERVERS:
    NATS_SERVERS = config("NATS_BROKER", default="nats://nats.wandelbots.svc:4222", cast=str)


# TODO: we need to create this at runtime, because the cell is dynmamic, inject to us via env var
NATS_SUBJECT_CYCLE = config("NATS_SUBJECT_CYCLE", default="cell.process.cycle")
NATS_SUBJECT_PROGRAM = config("NATS_SUBJECT_PROGRAM", default="cell.process.program")

_nats_client: nats.NATS | None = None
_program_state_queue: asyncio.Queue[str] = asyncio.Queue()
_program_state_consumer_task: Optional[asyncio.Task] = None


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


async def flush():
    global _nats_client
    if _nats_client:
        logger.debug("Flushing NATS client")
        await _nats_client.flush()


async def cycle_event_handler(sender, message: BaseCycleEvent, **kwargs):
    event_type = message.event_type
    logger.debug(f"NATS event handler received {event_type} event")
    nats_client = await get_client()
    try:
        await nats_client.publish(NATS_SUBJECT_CYCLE, message.model_dump_json().encode())
        logger.debug(f"Published {event_type} event to NATS")
    except Exception as e:
        logger.error(f"Failed to publish {event_type} event to NATS: {e}")


def program_state_change_handler(event: ProgramStateChangeEvent):
    """Report program state changes to NATS."""
    try:
        if _program_state_consumer_task is None:
            logger.warning("Program state change is queued but consumer task is not running")
            return

        if _program_state_consumer_task.done():
            logger.warning("Program state change is queued but consumer task is done")
            return

        data = event.model_dump_json()
        _program_state_queue.put_nowait(data)
    except Exception as e:
        logger.error(f"Failed to add program state change to queue: {e}, event is lost: {event}")


async def start_program_state_consumer():
    """Start the program state consumer task."""
    global _program_state_consumer_task, nats_client

    if _program_state_consumer_task is None or _program_state_consumer_task.done():
        logger.info("Starting NATS program state consumer")
        try:
            _program_state_consumer_task = asyncio.create_task(_program_state_consumer())
        except Exception as e:
            logger.error(f"Failed to start NATS program state consumer: {e}")
    else:
        logger.debug("NATS program state consumer already running")


async def _program_state_consumer():
    """Consume all program state data from the queue and publish it to NATS."""
    try:
        while True:
            data = await _program_state_queue.get()

            try:
                nats_client = await get_client()
                await nats_client.publish(NATS_SUBJECT_PROGRAM, data.encode())
                logger.debug("Published program state change to NATS")
            except asyncio.CancelledError:
                # allow cancellation
                raise
            except Exception as e:
                logger.error(f"Failed to publish program state change to NATS: {e}")

                # don't exhaust the event loop
                await asyncio.sleep(1)
    except asyncio.CancelledError:
        logger.info("Clearing remaining items in program state queue")
        while not _program_state_queue.empty():
            data = _program_state_queue.get_nowait()
            try:
                await _nats_client.publish(NATS_SUBJECT_PROGRAM, data)
            except Exception as e:
                logger.error(f"Failed to publish remaining program state change to NATS: {e}")

        logger.debug("Program state consumer cancelled")


async def stop_program_state_consumer():
    """Stop the program state consumer task and clear remaining queue items."""
    global _program_state_consumer_task

    if _program_state_consumer_task and not _program_state_consumer_task.done():
        logger.info("Stopping NATS program state consumer")
        _program_state_consumer_task.cancel()

        try:
            await _program_state_consumer_task
        except asyncio.CancelledError:
            pass

        _program_state_consumer_task = None
    else:
        logger.debug("NATS program state consumer not running")


if NATS_SERVERS:
    for signal in [cycle_started, cycle_finished, cycle_failed]:
        event_type = signal.name
        logger.debug(f"Connecting NATS event handler for {event_type} event")
        signal.connect(cycle_event_handler)
