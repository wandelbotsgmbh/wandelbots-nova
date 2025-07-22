import asyncio
from typing import Any, Generic, TypeVar

import nats
from decouple import config
from nats import NATS
from nats.js import JetStreamContext
from nats.js.api import KeyValueConfig
from nats.js.client import KeyValue
from nats.js.errors import KeyNotFoundError as KvKeyError
from nats.js.errors import NoKeysError, NotFoundError
from pydantic import BaseModel, Field, ValidationError, constr

from nova.core.logging import logger

# generally people use NATS_SERVERS, app store uses NATS_BROKERS
NATS_SERVERS = config("NATS_SERVERS", default=None)
if not NATS_SERVERS:
    NATS_SERVERS = config("NATS_BROKER", default="nats://nats.wandelbots.svc:4222", cast=str)

# this is optional, but needed when you connect to NATS server running in a portal instance from your local machine
NATS_TOKEN = config("NOVA_ACCESS_TOKEN", default=None)

T = TypeVar("T", bound=BaseModel)


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
        nats_client_config: dict | None = None,
        nats_kv_config: KeyValueConfig | None = None,
    ):
        """Initialize the KeyValueStore.

        Args:
            model_class: The Pydantic model class that will be stored in the KV store.
                        All stored objects must be instances of this class.
            nats_bucket_name: The name of the NATS JetStream bucket to use for storage.
                             If the bucket doesn't exist, it will be created if nats_kv_config
                             is provided.
            nats_client_config: Optional configuration dictionary for the NATS client connection.
                               If not provided, defaults will be used. Common options include:
                               - "servers": List of NATS server URLs (if not provided, will be read
                                 from NATS_SERVERS environment variables)
            nats_kv_config: Optional KeyValueConfig for creating the bucket if it doesn't exist.
                           If None and the bucket doesn't exist, an error will be raised.
                           Only required when creating new buckets.

        Raises:
            RuntimeError: If the bucket doesn't exist and no nats_kv_config is provided.
        """
        self._model_class = model_class
        self._nats_bucket_name = nats_bucket_name

        self._nats_client_config = nats_client_config or {}
        self._nats_kv_config = nats_kv_config

        self._nc: NATS | None = None
        self._js: JetStreamContext
        self._kv: KeyValue
        self._bucket_lock = asyncio.Lock()

    async def connect(self) -> None:
        """Connect to NATS and initialize JetStream Bucket"""
        if self._nc and self._nc.is_connected:
            return

        config = self._nats_client_config.copy()
        if "servers" not in config and NATS_SERVERS:
            config["servers"] = NATS_SERVERS

        if "token" not in config and NATS_TOKEN:
            config["token"] = NATS_TOKEN

        logger.info(f"Connecting to nats server: {config.get('servers')}")
        self._nc = await nats.connect(**config)
        self._js = self._nc.jetstream()
        logger.info("Connected to NATS")

    async def shutdown(self) -> None:
        """Disconnect from NATS"""
        if self._nc and self._nc.is_connected:
            await self._nc.drain()

    @property
    def is_connected(self) -> bool:
        """Check if client is connected"""
        return self._nc is not None and self._nc.is_connected

    @property
    async def kv(self) -> KeyValue:
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
        kv = await self.kv
        await kv.put(key, model.model_dump_json().encode())

    async def delete(self, key: str) -> None:
        """Delete a key from NATS KV store"""
        kv = await self.kv
        try:
            await kv.delete(key)
        except KvKeyError:
            pass

    async def get(self, key: str) -> T | None:
        """Get a specific model from NATS KV store"""
        kv = await self.kv
        try:
            entry = await kv.get(key)
            if entry.value is None:
                return None

            return self._model_class.model_validate_json(entry.value.decode())
        except (KvKeyError, ValidationError):
            return None

    async def get_all(self) -> list[T]:
        """Get all models from NATS KV store"""
        kv = await self.kv
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

    async def __aenter__(self):
        """Async context manager entry - connects to NATS"""
        await self.connect()
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        """Async context manager exit - disconnects from NATS"""
        await self.shutdown()
        return False


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
    def __init__(self, nats_bucket_name, nats_client_config=None, nats_kv_config=None):
        super().__init__(Program, nats_bucket_name, nats_client_config, nats_kv_config)
