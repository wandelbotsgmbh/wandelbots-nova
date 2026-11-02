"""Camera lifecycle management for policy execution."""

from __future__ import annotations

import asyncio
from typing import TYPE_CHECKING

from novapolicy.cameras.protocol import LatestFrameSource

if TYPE_CHECKING:
    from novapolicy.cameras.protocol import CameraFrame, CameraSource

# Frames feeding the Rerun view are held to a far looser bound than frames
# feeding the policy. Visualization does not need control-loop freshness, and a
# stale-frame error raised from the logging task would destabilise a run that is
# otherwise fine.
_LOGGING_MAX_AGE_S = 30.0


class CameraManager:
    """Connects, reads from, and disconnects a set of camera sources.

    ``max_age_s`` is the default freshness bound for every source. A channel
    that declares its own — because that camera runs at a different rate —
    overrides it through ``max_age_s_overrides``.
    """

    def __init__(self, max_age_s: float) -> None:
        self._max_age_s = max_age_s
        self._sources: dict[str, CameraSource] = {}
        self._overrides: dict[str, float] = {}

    @property
    def active(self) -> bool:
        return bool(self._sources)

    @property
    def names(self) -> list[str]:
        return list(self._sources)

    def max_age_for(self, key: str) -> float:
        """Freshness bound applied to one channel."""
        return self._overrides.get(key, self._max_age_s)

    async def connect(
        self,
        sources: dict[str, CameraSource],
        max_age_s_overrides: dict[str, float] | None = None,
    ) -> None:
        """Connect all camera sources concurrently."""
        self._overrides = dict(max_age_s_overrides or {})
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
        self._overrides.clear()

    def read(self) -> dict[str, CameraFrame]:
        """Read one policy frame from each camera source.

        Raises:
            RuntimeError: A source has no frame, or its newest frame is older
                than the bound for that channel.
        """
        return {
            key: source.read(max_age_s=self.max_age_for(key))
            for key, source in self._sources.items()
        }

    def read_latest_frames(self) -> dict[str, CameraFrame]:
        """Read the latest frame from camera sources that expose one.

        Feeds visualization, so it uses the loose logging bound rather than the
        control-loop one.
        """
        return {
            key: source.get_latest_frame(max_age_s=_LOGGING_MAX_AGE_S)
            for key, source in self._sources.items()
            if isinstance(source, LatestFrameSource)
        }
