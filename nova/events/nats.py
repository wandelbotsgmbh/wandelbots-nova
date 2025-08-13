import asyncio
import logging
import sys
from typing import Any, Generic, TypeVar

from decouple import config
from nats import NATS
from nats.js.api import KeyValueConfig
from nats.js.client import JetStreamContext, KeyValue
from nats.js.errors import KeyNotFoundError as KvKeyError
from nats.js.errors import NoKeysError, NotFoundError
from pydantic import BaseModel, Field, ValidationError, constr

# Can't import from nova.core because of cyclic imports
# need to refactor probably
# nova.core.gateway needs this
LOG_LEVEL: str = config("LOG_LEVEL", default="INFO").upper()
LOG_FORMAT: str = config("LOG_FORMAT", default="%(asctime)s [%(levelname)s] %(name)s: %(message)s")
LOG_DATETIME_FORMAT: str = config("LOG_DATETIME_FORMAT", default="%Y-%m-%d %H:%M:%S")
LOGGER_NAME: str = config("LOGGER_NAME", default="wandelbots-nova")

# Setting up the underlying logger
formatter: logging.Formatter = logging.Formatter(LOG_FORMAT, datefmt=LOG_DATETIME_FORMAT)
handler: logging.StreamHandler = logging.StreamHandler(sys.stdout)
handler.setLevel(LOG_LEVEL)
handler.setFormatter(formatter)

logger: logging.Logger = logging.getLogger(LOGGER_NAME)
logger.setLevel(LOG_LEVEL)
logger.addHandler(handler)


class Message:
    # TODO: we can add support for other NATS publishing params
    # e.g. message headers, reply
    # see nats.publish
    def __init__(self, subject: str, data: bytes):
        self.subject = subject
        self.data = data


class Client:
    """
    A wrapper around nats package, don't use nats package directly in the project.
    Instead use this client, so we can change it later.
    """

    def __init__(self, nats_servers: str):
        self._nats_servers = nats_servers
        self._nats_client: NATS = None

    async def connect(self):
        """Connect method for consistency with context manager pattern."""
        # Client is already initialized with the NATS client, so nothing to do here
        self._nats_client = NATS()
        await self._nats_client.connect(self._nats_servers)
        logger.debug("NATS client is ready")
        return self

    async def close(self):
        """Close the NATS client and clean up resources."""
        await self._nats_client.drain()
        logger.debug("NATS client closed")

    async def __aenter__(self):
        """Async context manager entry."""
        await self.connect()
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        """Async context manager exit."""
        await self.close()

    async def publish(self, message: Message):
        await self._nats_client.publish(message.subject, message.data)


class Publisher:
    def __init__(self, nats_client: Client):
        """
        Publishes messages to NATS with a background task.
        Uses a queue to manage publishing tasks.
        """
        self._nats_client = nats_client
        self._publish_queue = asyncio.Queue()
        self._publish_queue_consumer = asyncio.create_task(self._publish_queue_task())

    async def connect(self):
        """Connect method for consistency with context manager pattern."""
        # Publisher is already initialized with the NATS client, so nothing to do here
        logger.debug("NATS publisher is ready")
        return self

    async def close(self):
        """Close the NATS publisher and clean up resources."""
        await self._stop_nats_message_consumer()
        logger.debug("NATS publisher closed")

    async def __aenter__(self):
        """Async context manager entry."""
        await self.connect()
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        """Async context manager exit."""
        await self.close()

    def publish(self, message: Message):
        self._publish_queue.put_nowait(message)

    async def _stop_nats_message_consumer(self):
        """Stop the NATS message consumer task and clear remaining queue items."""
        if self._publish_queue_consumer and not self._publish_queue_consumer.done():
            logger.info("Stopping NATS message consumer")
            self._publish_queue_consumer.cancel()

            try:
                await self._publish_queue_consumer
            except asyncio.CancelledError:
                pass

            self._publish_queue_consumer = None
        else:
            logger.debug("NATS message consumer not running")

    async def _publish_queue_task(self):
        """Consume all program state data from the queue and publish it to NATS."""
        # TODO: double check this logic
        try:
            while True:
                nats_message = await self._publish_queue.get()
                logger.info("publishing")

                try:
                    await self._nats_client._nats_client.publish(nats_message)
                except asyncio.CancelledError:
                    # allow cancellation
                    raise
                except Exception as e:
                    logger.error(f"Failed to publish program state change to NATS: {e}")

                    # don't exhaust the event loop
                    await asyncio.sleep(1)
        except asyncio.CancelledError:
            logger.info("Clearing remaining items in nats publishing queue")
            while not self._publish_queue.empty():
                message = self._publish_queue.get_nowait()
                try:
                    await self._nats_client.publish(message.subject, message.data)
                except Exception as e:
                    logger.error(f"Failed to publish remaining messages to NATS: {e}")

            logger.info("NATS message consumer cancelled")


T = TypeVar("T", bound=BaseModel)


# We don't want to expose this to public usage until the jetstream concept gets more mature
class _KeyValueStore(Generic[T]):
    """Generic NATS-backed key-value store for Pydantic models

    This class provides a convenient interface for storing and retrieving Pydantic models
    in a NATS JetStream Key-Value store. It handles connection management, serialization,
    and provides type-safe operations.

    Note: KeyValueConfig is only needed if you want to create the bucket when it doesn't exist.
    If the bucket already exists, you can simply use the bucket name.

    Example (simple usage with existing bucket):
        ```python
        # Define your Pydantic model
        class Program(BaseModel):
            name: str

        async with KeyValueStore(Program, "programs") as store:
            # Store a program
            program = Program(name="My Robot Program")
            await store.put("program:1", program)

            retrieved_program = await store.get("program:1")
        ```

    Example (with bucket creation):
        ```python
        from nats.js.api import KeyValueConfig

        # Create with bucket creation config
        kv_config = KeyValueConfig(bucket="programs")
        async with KeyValueStore(
            model_class=Program,
            nats_bucket_name="programs",
            nats_kv_config=kv_config
        ) as store:
            program = Program(name="Auto-created bucket example")
            await store.put("program:auto", program)
        ```
    """

    def __init__(
        self,
        model_class: type[T],
        nats_bucket_name: str,
        nats_client: Client,
        nats_kv_config: KeyValueConfig | None = None,
    ):
        """Initialize the KeyValueStore.

        Args:
            model_class: The Pydantic model class that will be stored in the KV store.
                        All stored objects must be instances of this class.
            nats_bucket_name: The name of the NATS JetStream bucket to use for storage.
                             If the bucket doesn't exist, it will be created if nats_kv_config
                             is provided.
            nats_client: The NATS client instance to use for communication with the NATS server.
            nats_kv_config: Optional KeyValueConfig for creating the bucket if it doesn't exist.
                           If None and the bucket doesn't exist, an error will be raised.
                           Only required when creating new buckets.

        Raises:
            RuntimeError: If the bucket doesn't exist and no nats_kv_config is provided.
        """
        self._model_class = model_class
        self._nats_bucket_name = nats_bucket_name

        self._nats_kv_config = nats_kv_config

        self._nc: NATS = nats_client
        self._js: JetStreamContext = self._nc.jetstream()
        self._kv: KeyValue
        self._bucket_lock = asyncio.Lock()

    @property
    def is_connected(self) -> bool:
        """Check if client is connected"""
        return self._nc is not None and self._nc.is_connected

    @property
    async def _key_value(self) -> KeyValue:
        """Get the KeyValue store, connecting and creating bucket if necessary.

        If the bucket doesn't exist and no nats_kv_config was provided during initialization,
        a RuntimeError will be raised. If nats_kv_config was provided, the bucket will be
        created automatically.

        Returns:
            KeyValue: The NATS JetStream KeyValue store instance.

        Raises:
            RuntimeError: If the bucket doesn't exist and no nats_kv_config was provided.
        """
        if not self.is_connected:
            await self.connect()

        if getattr(self, "_kv", None) is not None:
            return self._kv

        async with self._bucket_lock:
            try:
                self._kv = await self._js.key_value(self._nats_bucket_name)
            except NotFoundError:
                if not self._nats_kv_config:
                    raise RuntimeError(
                        f"Bucket {self._nats_bucket_name} missing and no kv_config supplied"
                    )
                self._kv = await self._js.create_key_value(self._nats_kv_config)

        return self._kv

    async def put(self, key: str, model: T) -> None:
        """Store a Pydantic model in NATS KV store"""
        kv = await self._key_value
        await kv.put(key, model.model_dump_json().encode())

    async def delete(self, key: str) -> None:
        """Delete a key from NATS KV store"""
        kv = await self._key_value
        try:
            await kv.delete(key)
        except KvKeyError:
            pass

    async def get(self, key: str) -> T | None:
        """Get a specific model from NATS KV store"""
        kv = await self._key_value
        try:
            entry = await kv.get(key)
            if entry.value is None:
                return None

            return self._model_class.model_validate_json(entry.value.decode())
        except (KvKeyError, ValidationError):
            return None

    async def get_all(self) -> list[T]:
        """Get all models from NATS KV store"""
        kv = await self._key_value
        try:
            keys = await kv.keys()
        except NoKeysError:
            return []

        models: list[T] = []
        for key in keys:
            try:
                entry = await kv.get(key)
                if entry.value is None:
                    continue

                model = self._model_class.model_validate_json(entry.value.decode())
                models.append(model)
            except KvKeyError:
                logger.error(f"Key {key} not found in KV store")
            except ValidationError:
                logger.error(f"Validation error for key {key}, skipping")
                continue

        return models


# this is data model that we should take from service manager api client
# for now we are duplicating the model here, will be removed once the other side is ready
class Program(BaseModel):
    program: constr(pattern=r"^[a-zA-Z0-9_-]+$", min_length=1, max_length=255) = Field(  # type: ignore
        ..., examples=["my_program"], title="Unique program identifier"
    )
    name: str | None = Field(None, title="Program name")
    description: str | None = Field(None, title="Program description")
    app: str = Field(..., title="The app containing the program.")
    input_schema: dict[str, Any] | None = Field(None, title="Program input json schema")
    preconditions: dict[str, Any] | None = Field(
        None, title="Preconditions before the program can be started"
    )


# ProgramStore = KeyValueStore[Program] would be better but python doesn't support this
# when I do store = ProgramStore() the __orig_class__ is not available in the __init__
# my reseach say's python doesn't capture the type argument when I do this


# TODO: change the Program with wandelbots_api_client.v2.models.Program
class ProgramStore(_KeyValueStore[Program]):
    def __init__(
        self, nats_bucket_name: str, nats_client: Client, nats_kv_config: KeyValueConfig = None
    ):
        super().__init__(Program, nats_bucket_name, nats_client, nats_kv_config)
