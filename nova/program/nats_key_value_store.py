import os
from typing import TypeVar, Generic, Type, get_args
from decouple import config

import nats
from nats import NATS
from nats.js import JetStreamContext
from nats.js.api import KeyValueConfig
from nats.js.client import KeyValue
from nats.js.errors import KeyNotFoundError as KvKeyError
from nats.js.errors import NotFoundError, NoKeysError
from nova.core.logging import logger

from pydantic import BaseModel, ValidationError

from nova.program.function import Program

# generally people use NATS_SERVERS, app store uses NATS_BROKERS
NATS_SERVERS = config("NATS_SERVERS", default=None, cast=str)
if not NATS_SERVERS:
    NATS_SERVERS = config("NATS_BROKERS", default="nats://nats.wandelbots.svc:4222", cast=str)

# NATS_TOKEN is optional, but needed when you connect remotely to a NATS server
NATS_TOKEN = config("NATS_TOKEN", default=None, cast=str)

T = TypeVar('T', bound=BaseModel)
class KeyValueStore(Generic[T]):
    """Generic NATS-backed key-value store for Pydantic models"""
    
    def __init__(
            self,
            model_class: Type[T],
            nats_bucket_name: str,
            nats_client_config: dict | None = None,
            nats_kv_config: KeyValueConfig | None = None,
        ):
        self.model_class = model_class
        self.kv_bucket_name = nats_bucket_name

        self.nats_client_config = nats_client_config or {}
        self.kv_config = nats_kv_config or {}

        self.nc: NATS | None = None
        self.js: JetStreamContext | None = None
        self.kv: KeyValue | None = None

    async def connect(self) -> None:
        """Connect to NATS and initialize JetStream Bucket"""
        if self.nc and self.nc.is_connected:
            return

        # Start with user config
        config = self.nats_client_config.copy()
        
        # Add servers if not provided by user but available in environment
        if "servers" not in config and NATS_SERVERS:
            config["servers"] = NATS_SERVERS
            
        # Add token if not provided by user but available in environment
        if "token" not in config and NATS_TOKEN:
            config["token"] = NATS_TOKEN

        logger.info(f"Connecting to nats server: {config.get('servers')}")
        self.nc = await nats.connect(**config)
        self.js = self.nc.jetstream()

        self.kv = await self._create_kv_bucket()
        logger.info(f"Connected to NATS and initialized KV bucket '{self.kv_bucket_name}'")

    async def _create_kv_bucket(self) -> None:
        try:
            kv = await self.js.key_value(bucket=self.kv_bucket_name)
            logger.info(f"Using already existing KV bucket '{self.kv_bucket_name}'")
        except NotFoundError:
            if not self.kv_config:
                raise Exception(f"The bucket with name: {self.kv_bucket_name} doesn't exists and kv_config is not provided, so it cannot be created.")

            logger.info(f"No bucket found, creating new KV bucket '{self.kv_bucket_name}'")
            kv = await self.js.create_key_value(config=self.kv_config)
        return kv

    async def shutdown(self) -> None:
        """Disconnect from NATS"""
        if self.nc and self.nc.is_connected:
            await self.nc.drain()

    @property
    def is_connected(self) -> bool:
        """Check if client is connected"""
        return self.nc is not None and self.nc.is_connected

    async def put(self, key: str, model: T) -> None:
        """Store a Pydantic model in NATS KV store"""
        if not self.kv:
            await self._get_or_create_kv_bucket()
        
        await self.kv.put(key, model.model_dump_json().encode())

    async def delete(self, key: str) -> None:
        """Delete a key from NATS KV store"""
        if not self.kv:
            await self._get_or_create_kv_bucket()
        
        try:
            await self.kv.delete(key)
        except KvKeyError:
            pass

    async def get(self, key: str) -> T | None:
        """Get a specific model from NATS KV store"""
        if not self.kv:
            await self._get_or_create_kv_bucket()

        try:
            entry = await self.kv.get(key)
            return self.model_class.model_validate_json(entry.value.decode())
        except (KvKeyError, ValidationError):
            return None

    async def get_all(self) -> list[T]:
        """Get all models from NATS KV store"""
        if not self.kv:
            await self._get_or_create_kv_bucket()

        try:
            keys = await self.kv.keys()
        except NoKeysError:
            return []
        
        models: list[T] = []
        for key in keys:
            try:
                entry = await self.kv.get(key)
                model = self.model_class.model_validate_json(entry.value.decode())
                models.append(model)
            except KvKeyError:
                logger.error(f"Key {key} not found in KV store")
            except ValidationError:
                logger.error(f"Validation error for key {key}, skipping")
                continue
                
        return models