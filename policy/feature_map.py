"""Feature mapping between LeRobot flat feature dicts and NOVA motion groups."""

from __future__ import annotations

import asyncio
import contextlib
import logging
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from nova.cell.motion_group import MotionGroup
    from nova.types import RobotState

logger = logging.getLogger(__name__)


@dataclass
class FeatureGroup:
    """Maps a motion group to named features for the policy.

    The name defines the prefix for flat feature keys:
    - joints: ``{name}_joint_{i}.pos`` (e.g. ``left_joint_1.pos``)
    - IOs: ``{name}_{io_name}`` (e.g. ``left_gripper``, ``left_conveyor_sensor``)
    """

    motion_group: MotionGroup
    """The NOVA motion group this feature group controls."""

    name: str
    """Name prefix for feature keys, e.g. ``left``, ``right``, ``arm_0``."""

    ios: dict[str, str] | None = None
    """Named IO mapping: feature name → hardware IO key.

    Example: ``{"gripper": "digital_out[0]", "conveyor_sensor": "digital_in[0]"}``

    In observations, the policy sees ``left_gripper``, ``left_conveyor_sensor``.
    In actions, if the policy outputs ``left_gripper: 1.0``, it writes to ``digital_out[0]``.
    """

    io_threshold: float = 0.5
    """For boolean IOs: policy values >= threshold write True, below write False."""

    def joint_key(self, joint_index: int) -> str:
        """Return the flat feature key for a 1-based joint index."""
        return f"{self.name}_joint_{joint_index}.pos"

    def io_feature_key(self, io_name: str) -> str:
        """Return the flat feature key for a named IO."""
        return f"{self.name}_{io_name}"

    @property
    def io_hardware_keys(self) -> list[str]:
        """All hardware IO keys to stream."""
        if self.ios is None:
            return []
        return list(self.ios.values())


class _IOStreamCache:
    """Caches the latest IO values via a persistent WebSocket stream."""

    def __init__(self, motion_group: MotionGroup, ios: list[str]) -> None:
        self._mg = motion_group
        self._io_keys = ios
        self._values: dict[str, object] = {}
        self._task: asyncio.Task[None] | None = None
        self._ready = asyncio.Event()

    @property
    def values(self) -> dict[str, object]:
        return self._values

    async def start(self) -> None:
        """Start the background IO stream listener."""
        if self._task is not None:
            return
        self._task = asyncio.create_task(self._run(), name=f"io-stream-{self._mg.id}")
        try:
            await asyncio.wait_for(self._ready.wait(), timeout=5.0)
        except TimeoutError:
            logger.warning("IO stream for %s did not receive initial values within 5s", self._mg.id)

    async def stop(self) -> None:
        """Cancel the background stream task."""
        if self._task is not None:
            self._task.cancel()
            with contextlib.suppress(asyncio.CancelledError, OSError):
                await self._task
            self._task = None

    async def _run(self) -> None:
        """Subscribe to IO stream and cache values."""
        api_client = self._mg._api_client
        cell = self._mg._cell
        controller_id = self._mg._controller_id

        if not self._io_keys:
            self._ready.set()
            return

        try:
            stream = api_client.controller_ios_api.stream_io_values(
                cell=cell,
                controller=controller_id,
                ios=self._io_keys,
            )
            async for response in stream:
                for io_val in response.io_values:
                    self._values[io_val.root.io] = io_val.root.value
                if not self._ready.is_set():
                    self._ready.set()
        except asyncio.CancelledError:
            raise
        except (OSError, RuntimeError) as e:
            logger.warning("IO stream for %s ended: %s", self._mg.id, e)
        except Exception:
            logger.exception("IO stream for %s crashed", self._mg.id)


@dataclass
class FeatureMap:
    """Maps between flat feature dicts and NOVA motion groups.

    When ``start()`` / ``stop()`` are called (or used as async context manager),
    opens WebSocket IO streams for each controller.
    """

    groups: list[FeatureGroup]
    _io_caches: list[_IOStreamCache] = field(default_factory=list, init=False, repr=False)

    def get_motion_groups(self) -> list[MotionGroup]:
        """Get all motion groups in order."""
        return [group.motion_group for group in self.groups]

    async def start(self) -> None:
        """Open IO streams for all controllers referenced by the feature groups."""
        controller_ios: dict[str, set[str]] = {}
        controller_mg: dict[str, object] = {}
        for group in self.groups:
            controller_id = group.motion_group._controller_id
            if controller_id not in controller_ios:
                controller_ios[controller_id] = set()
                controller_mg[controller_id] = group.motion_group
            controller_ios[controller_id].update(group.io_hardware_keys)

        for controller_id, io_keys in controller_ios.items():
            if not io_keys:
                continue
            mg = controller_mg[controller_id]
            cache = _IOStreamCache(mg, ios=sorted(io_keys))
            self._io_caches.append(cache)
            await cache.start()

    async def stop(self) -> None:
        """Close all IO streams."""
        for cache in self._io_caches:
            await cache.stop()
        self._io_caches.clear()

    async def __aenter__(self) -> FeatureMap:
        await self.start()
        return self

    async def __aexit__(self, *exc: object) -> None:
        await self.stop()

    async def build_observation(self, states: dict[str, RobotState]) -> dict[str, float]:
        """Convert per-group RobotState into a flat feature dict."""
        obs: dict[str, float] = {}
        for group in self.groups:
            state = states.get(group.motion_group.id)
            if state is None:
                continue

            # Joints
            for joint_index, joint_value in enumerate(state.joints, start=1):
                obs[group.joint_key(joint_index)] = joint_value

            # IOs — read from stream cache
            if group.ios is None:
                continue
            cache = self._get_cache(group.motion_group.id)
            if cache is None:
                continue
            for name, hw_key in group.ios.items():
                raw_value = cache.values.get(hw_key)
                if raw_value is None:
                    obs[group.io_feature_key(name)] = 0.0
                elif isinstance(raw_value, bool):
                    obs[group.io_feature_key(name)] = 1.0 if raw_value else 0.0
                else:
                    obs[group.io_feature_key(name)] = float(raw_value)

        return obs

    def parse_action(self, action: dict[str, float]) -> tuple[
        dict[str, list[list[float]]],
        dict[str, dict[str, bool | int | float | str]] | None,
    ]:
        """Convert a flat action dict into per-group joints and IOs."""
        joints: dict[str, list[list[float]]] = {}
        ios: dict[str, dict[str, bool | int | float | str]] = {}

        for group in self.groups:
            motion_group_id = group.motion_group.id

            # Joints — collect as many as are present in the action
            joint_values: list[float] = []
            joint_index = 1
            while True:
                key = group.joint_key(joint_index)
                if key not in action:
                    break
                joint_values.append(float(action[key]))
                joint_index += 1

            if joint_values:
                joints[motion_group_id] = [joint_values]

            # IOs
            if group.ios is None:
                continue
            group_ios: dict[str, bool | int | float | str] = {}
            for name, hw_key in group.ios.items():
                feature_key = group.io_feature_key(name)
                if feature_key not in action:
                    continue
                value = float(action[feature_key])
                # Boolean IOs: threshold
                group_ios[hw_key] = value >= group.io_threshold

            if group_ios:
                ios[motion_group_id] = group_ios

        return joints, ios or None

    def _get_cache(self, motion_group_id: str) -> _IOStreamCache | None:
        """Find the IO cache for a given motion group's controller."""
        for cache in self._io_caches:
            if cache._mg.id == motion_group_id or cache._mg._controller_id == motion_group_id.rsplit("@", maxsplit=1)[-1]:
                return cache
        return None
