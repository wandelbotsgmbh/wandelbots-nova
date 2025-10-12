import pytest
import asyncio

from typing import Optional
from loguru import logger
from nova.core.nova import Nova
from nova.program.runner import ProgramRun, ProgramRunState
from nova.program.store import _KeyValueStore
from nova.program.store import _KeyValueStore
from nats.js.api import KeyValueConfig
from nova.nats import NatsClient
from nats import NATS
from pydantic import BaseModel


@pytest.mark.asyncio
async def test_delete_bucket_and_stream():
    """DELETE THE NATS RESOURCES SO WE CAN HAVE A FRESH START WHEN A BUG HAPPENS"""

    async with Nova() as nova:
        raw_client: NATS =  nova.nats.raw_client()
        jet_stream_context = raw_client.jetstream()

        # delete the bucket if it exists
        deleted_kv = await jet_stream_context.delete_key_value("nova_v2_cells_cell_program_status")
        logger.info(f"Deleted kv: {deleted_kv}")

        # delete the stream if it exists
        deleted_stream = await jet_stream_context.delete_stream("program_runs")
        logger.info(f"Deleted stream: {deleted_stream}")



class ProgramStatus(BaseModel):
    program: str
    app: Optional[str] = None
    state: ProgramRunState


class ProgramStatusStore(_KeyValueStore[ProgramStatus]):
    def __init__(self, cell_id: str, nats_client: NatsClient, create_bucket: bool = False):
        self._nats_bucket_name = "nova_v2_cells_{cell}_program_status".format(cell=cell_id)
        self._kv_config = KeyValueConfig(
            bucket=self._nats_bucket_name
        )

        super().__init__(
            ProgramStatus,
            nats_bucket_name=self._nats_bucket_name,
            nats_client=nats_client,
            nats_kv_config=self._kv_config if create_bucket else None,
        )


@pytest.mark.asyncio
async def test_create_stream_and_bucket():
    """CREATE NATS RESOURCES FOR THE DEMO"""

    async with Nova() as nova:
        raw_client: NATS =  nova.nats._nats_client
        jet_stream_context = raw_client.jetstream()

        stream_info = await jet_stream_context.add_stream(name="program_runs", subjects=["nova.v2.cells.cell.programs"])
        logger.info(f"Created stream: {stream_info}")

        program_status_store = ProgramStatusStore(cell_id="cell", nats_client=nova.nats, create_bucket=True)
        all_data = await program_status_store.get_all()  # bucket creation is lazy, so we need to call something to make sure the bucket is created



@pytest.mark.asyncio
async def test_consume_program_run():
    """CREATE JETSTREAM CONSUMER WHICH LISTENS PROGRAM RUN MESSAGES AND STORES THE LATEST STATE IN A KEY-VALUE BUCKET"""
    async with Nova() as nova:
        raw_client: NATS =  nova.nats._nats_client
        jet_stream_context = raw_client.jetstream()
        from nats.js.api import ConsumerConfig, DeliverPolicy, AckPolicy
        program_status_store = ProgramStatusStore(cell_id="cell", nats_client=nova.nats, create_bucket=False)


        async def program_run_message_received(msg):
            program_run = ProgramRun.model_validate_json(msg.data)
            logger.info(f"Received program run message for program: {program_run.app}.{program_run.program} state: {program_run.state}")
            await program_status_store.put(
                f"{program_run.app}.{program_run.program}",
                ProgramStatus(
                    state=program_run.state,
                    app=program_run.app,
                    program=program_run.program,
                )
            )
            await msg.ack()


        try:
            subscription = await jet_stream_context.subscribe(
                subject="nova.v2.cells.cell.programs",
                durable="program_operator", # this is the name of the consumer, NATS server will remember this consumer and store a cursor for it, even if consumer crashes it will catch up when it comes back online
                stream="program_runs",
                config=ConsumerConfig(
                    deliver_policy=DeliverPolicy.ALL,
                    ack_policy=AckPolicy.EXPLICIT, # we want to explicitly ack, so that if we crash before processing the message, we can process it again
                ),
                cb=program_run_message_received
            )

            while True:
                await asyncio.sleep(5)
        finally:
            await subscription.unsubscribe()


# https://github.com/nats-io/nats.py/pull/644
# TODO: check NATS key-value bucket performance and see if it fits to our use case
# OPERATION FIELD IS COMING IN EMPTY, SO WE CANNOT RELY ON IT, BUT IT GIVES A HINT TO FETCH THE LATEST VALUE
@pytest.mark.asyncio
async def test_consume_key_value_bucket():
    """CONSUME MESSAGES FROM THE KEY-VALUE BUCKET"""
    async with Nova() as nova:
        raw_client: NATS =  nova.nats._nats_client
        jet_stream_context = raw_client.jetstream()
        from nats.js.kv import KeyValue
        key_value: KeyValue = await jet_stream_context.key_value("nova_v2_cells_cell_program_status")
        watcher: KeyValue.KeyWatcher = await key_value.watchall()

        while True:
            update = await watcher._updates.get()
            if watcher._sub is None:
                logger.warning("Watcher subscription is None, retrying in 1 second")
                await asyncio.sleep(1)
                continue
            
            logger.info(f"Received key-value update: {update}")

