"""IO streaming and writing for policy execution.

Provides two classes:
- ``IOStreamCache``: background WebSocket that caches latest IO values
- ``IOWriter``: deduplicated IO writes with serialized access
"""

from __future__ import annotations

import asyncio
import contextlib
import logging
from typing import TYPE_CHECKING

from nova import api
from policy._sdk import get_api_gateway, get_cell, get_controller_id

if TYPE_CHECKING:
    from nova.cell.motion_group import MotionGroup
    from policy.types import ValueType

logger = logging.getLogger(__name__)


class IOStreamCache:
    """Caches the latest IO values via a persistent WebSocket stream."""

    def __init__(self, motion_group: MotionGroup, ios: list[str]) -> None:
        self.motion_group = motion_group
        self._io_keys = ios
        self.values: dict[str, object] = {}
        self._task: asyncio.Task[None] | None = None
        self._ready = asyncio.Event()

    async def start(self) -> None:
        """Start the background IO stream listener."""
        if self._task is not None:
            return
        self._task = asyncio.create_task(self._run(), name=f"io-stream-{self.motion_group.id}")
        try:
            await asyncio.wait_for(self._ready.wait(), timeout=5.0)
        except TimeoutError:
            logger.warning(
                "IO stream for %s did not receive initial values within 5s",
                self.motion_group.id,
            )

    async def stop(self) -> None:
        """Cancel the background stream task."""
        if self._task is not None:
            self._task.cancel()
            with contextlib.suppress(asyncio.CancelledError, OSError):
                await self._task
            self._task = None

    async def _run(self) -> None:
        """Subscribe to IO stream and cache values."""
        api_client = get_api_gateway(self.motion_group)
        cell = get_cell(self.motion_group)
        controller_id = get_controller_id(self.motion_group)

        if not self._io_keys:
            self._ready.set()
            return

        stream = None
        try:
            stream = api_client.controller_ios_api.stream_io_values(
                cell=cell,
                controller=controller_id,
                ios=self._io_keys,
            )
            async for response in stream:
                for io_val in response.io_values:
                    self.values[io_val.root.io] = io_val.root.value
                if not self._ready.is_set():
                    self._ready.set()
        except asyncio.CancelledError:
            raise
        except (OSError, RuntimeError) as e:
            logger.warning("IO stream for %s ended: %s", self.motion_group.id, e)
        except Exception:
            logger.exception("IO stream for %s crashed", self.motion_group.id)
        finally:
            if stream is not None:
                with contextlib.suppress(Exception):
                    await stream.aclose()


class IOWriter:
    """Writes IO values with deduplication and serialized access.

    Only writes values that have actually changed, and serializes writes
    per instance to avoid 429 rate-limit errors from the NOVA API.
    """

    def __init__(self, motion_group: MotionGroup) -> None:
        self._motion_group = motion_group
        self._last_written: dict[str, ValueType] = {}
        self._lock = asyncio.Lock()

    async def write(self, ios: dict[str, ValueType]) -> None:
        """Write IO values, skipping unchanged ones."""
        api_client = get_api_gateway(self._motion_group)
        cell = get_cell(self._motion_group)
        controller_id = get_controller_id(self._motion_group)

        for key, value in ios.items():
            if self._last_written.get(key) == value:
                continue
            async with self._lock:
                if self._last_written.get(key) == value:
                    continue
                try:
                    if isinstance(value, bool):
                        io_value = api.models.IOBooleanValue(io=key, value=value)
                    elif isinstance(value, (int, float)):
                        io_value = api.models.IOFloatValue(io=key, value=float(value))
                    else:
                        io_value = api.models.IOStringValue(io=key, value=str(value))

                    await api_client.controller_ios_api.set_output_values(
                        cell=cell, controller=controller_id, io_value=[io_value]
                    )
                    self._last_written[key] = value
                except (OSError, RuntimeError, ValueError, KeyError) as e:
                    logger.warning("Failed to write IO %s=%s: %s", key, value, e)
