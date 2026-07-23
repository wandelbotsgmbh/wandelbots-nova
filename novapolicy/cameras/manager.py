"""Camera lifecycle management for policy execution."""

from __future__ import annotations

import asyncio
from typing import TYPE_CHECKING

from novapolicy.cameras.protocol import LatestFrameSource

if TYPE_CHECKING:
    from novapolicy.cameras.protocol import CameraFrame, CameraSource


class CameraManager:
    """Connects, reads from, and disconnects a set of camera sources."""

    def __init__(self, max_age_s: float) -> None:
        self._max_age_s = max_age_s
        self._sources: dict[str, CameraSource] = {}

    @property
    def active(self) -> bool:
        return bool(self._sources)

    @property
    def names(self) -> list[str]:
        return list(self._sources)

    async def connect(self, sources: dict[str, CameraSource]) -> None:
        """Connect all camera sources concurrently."""
        tasks = []
        for key, source in sources.items():
            self._sources[key] = source
            tasks.append(source.connect())
        if tasks:
            await asyncio.gather(*tasks)

    async def disconnect(self) -> None:
        """Disconnect all camera sources."""
        tasks = [source.disconnect() for source in self._sources.values()]
        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)
        self._sources.clear()

    def read(self) -> dict[str, CameraFrame]:
        """Read one policy frame from each camera source."""
        return {
            key: source.read(max_age_s=self._max_age_s) for key, source in self._sources.items()
        }

    def read_latest_frames(self) -> dict[str, CameraFrame]:
        """Read the latest frame from camera sources that expose one."""
        return {
            key: source.get_latest_frame(max_age_s=self._max_age_s)
            for key, source in self._sources.items()
            if isinstance(source, LatestFrameSource)
        }
