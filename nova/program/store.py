import asyncio
from typing import Any, Generic, TypeVar

import nats
from nats.js.api import KeyValueConfig
from nats.js.client import JetStreamContext, KeyValue
from nats.js.errors import KeyNotFoundError as KvKeyError
from nats.js.errors import NoKeysError, NotFoundError
from pydantic import BaseModel, Field, ValidationError, constr

from nova.cell import Cell
from nova.logging import logger as nova_logger

_T = TypeVar("_T", bound=BaseModel)
_NATS_PROGRAMS_BUCKET_TEMPLATE = "nova_cells_{cell}_programs"
_NATS_PROGRAMS_MESSAGE_SIZE = 128 * 1024
_NATS_PROGRAMS_BUCKET_SIZE = _NATS_PROGRAMS_MESSAGE_SIZE * 100


# We don't want to expose this to public usage until the jetstream concept gets more mature
class _KeyValueStore(Generic[_T]):
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
        model_class: type[_T],
        nats_bucket_name: str,
        nats_client: nats.NATS,
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

        self._nats_client = nats_client
        self._nc: nats.NATS | None = None
        self._js: JetStreamContext | None = None
        self._kv: KeyValue | None = None
        self._bucket_lock = asyncio.Lock()
        self._logger = nova_logger.getChild("ProgramStore")

    @property
    def is_connected(self) -> bool:
        """Check if client is connected"""
        return self._nats_client is not None and self._nats_client.is_connected

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
            raise RuntimeError("NATS client is not connected. Call connect() first.")

        if self._kv is not None:
            return self._kv

        async with self._bucket_lock:
            # Initialize NATS and JetStream if not already done
            if self._nc is None:
                self._nc = self._nats_client
            if self._js is None:
                self._js = self._nc.jetstream()

            try:
                self._kv = await self._js.key_value(self._nats_bucket_name)
            except NotFoundError:
                if not self._nats_kv_config:
                    raise RuntimeError(
                        f"Bucket {self._nats_bucket_name} missing and no kv_config supplied"
                    )
                self._kv = await self._js.create_key_value(self._nats_kv_config)

        return self._kv

    async def put(self, key: str, model: _T) -> None:
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

    async def get(self, key: str) -> _T | None:
        """Get a specific model from NATS KV store"""
        kv = await self._key_value
        try:
            entry = await kv.get(key)
            if entry.value is None:
                return None

            return self._model_class.model_validate_json(entry.value.decode())
        except (KvKeyError, ValidationError):
            return None

    async def get_all(self) -> list[_T]:
        """Get all models from NATS KV store"""
        kv = await self._key_value
        try:
            keys = await kv.keys()
        except NoKeysError:
            return []

        models: list[_T] = []
        for key in keys:
            try:
                entry = await kv.get(key)
                if entry.value is None:
                    continue

                model = self._model_class.model_validate_json(entry.value.decode())
                models.append(model)
            except KvKeyError:
                self._logger.error(f"Key {key} not found in KV store")
            except ValidationError:
                self._logger.error(f"Validation error for key {key}, skipping")
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


class ProgramStore(_KeyValueStore[Program]):
    """
    Program store manages all the programs registered in a cell.
    """

    def __init__(self, cell: Cell, create_bucket: bool = False):
        self._cell = cell
        self._nats_bucket_name = _NATS_PROGRAMS_BUCKET_TEMPLATE.format(cell=cell.cell_id)
        self._kv_config = KeyValueConfig(
            bucket=self._nats_bucket_name,
            max_value_size=_NATS_PROGRAMS_MESSAGE_SIZE,
            max_bytes=_NATS_PROGRAMS_BUCKET_SIZE,
        )
        super().__init__(
            Program,
            nats_bucket_name=self._nats_bucket_name,
            # TODO: remove the NATS client wrapper
            nats_client=self._cell._api_gateway._nats_client,
            nats_kv_config=self._kv_config if create_bucket else None,
        )
