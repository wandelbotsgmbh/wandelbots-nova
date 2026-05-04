"""Mock action source for testing and development."""

from __future__ import annotations

import asyncio
import math
import time
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from nova_policy.types import ActionChunk

if TYPE_CHECKING:
    from collections.abc import AsyncIterator

    from nova_policy.types import ValueType


@dataclass
class MockActionSource:
    """Generates sinusoidal joint trajectories for testing the PolicyRunner.

    Implements AsyncIterator[ActionChunk] — drop-in replacement for a real policy.

    Usage:
        source = MockActionSource(
            motion_group_ids=["0@ur5e"],
            num_joints=6,
            home_joints=[0.0, -1.57, 1.57, 0.0, 1.57, 0.0],
            interval_ms=100,
            amplitude=0.2,
        )
        async for chunk in source:
            await runner.send(chunk)
    """

    motion_group_ids: list[str]
    """Motion group IDs to generate actions for."""

    num_joints: int = 6
    """Number of joints per motion group."""

    home_joints: list[float] = field(default_factory=lambda: [0.0] * 6)
    """Home joint positions (center of oscillation)."""

    interval_ms: int = 100
    """Emit interval in milliseconds."""

    amplitude: float = 0.2
    """Oscillation amplitude in radians."""

    frequency: float = 0.5
    """Oscillation frequency in Hz."""

    chunk_size: int = 1
    """Number of waypoints per chunk. >1 simulates policy-style multi-step chunks."""

    max_steps: int | None = None
    """Maximum number of chunks to emit. None = infinite."""

    io_toggle_key: str | None = None
    """Optional IO key to toggle periodically."""

    io_toggle_interval_ms: int = 2000
    """How often to toggle the IO (milliseconds)."""

    # Internal state
    _step_count: int = field(default=0, init=False, repr=False)
    _start_time: float = field(default=0.0, init=False, repr=False)
    _io_state: bool = field(default=False, init=False, repr=False)
    _last_io_toggle: float = field(default=0.0, init=False, repr=False)

    def __aiter__(self) -> AsyncIterator[ActionChunk]:
        """Return self as async iterator."""
        self._step_count = 0
        self._start_time = time.monotonic()
        self._last_io_toggle = self._start_time
        return self

    async def __anext__(self) -> ActionChunk:
        """Generate the next action chunk."""
        if self.max_steps is not None and self._step_count >= self.max_steps:
            raise StopAsyncIteration

        await asyncio.sleep(self.interval_ms / 1000.0)

        now = time.monotonic()
        elapsed = now - self._start_time

        # Generate joints for each step in the chunk
        joints: dict[str, list[list[float]]] = {}
        for group_idx, group_id in enumerate(self.motion_group_ids):
            steps: list[list[float]] = []
            for step_idx in range(self.chunk_size):
                step_time = elapsed + (step_idx * self.interval_ms / 1000.0)
                phase_offset = group_idx * 0.5  # Different phase per group
                joint_targets = self._generate_joints(step_time, phase_offset)
                steps.append(joint_targets)
            joints[group_id] = steps

        # Handle IO toggling
        ios: dict[str, dict[str, ValueType]] | None = None
        if self.io_toggle_key is not None:
            if (now - self._last_io_toggle) * 1000.0 >= self.io_toggle_interval_ms:
                self._io_state = not self._io_state
                self._last_io_toggle = now
                ios = {
                    group_id: {self.io_toggle_key: self._io_state}
                    for group_id in self.motion_group_ids
                }

        dt_ms = self.interval_ms if self.chunk_size > 1 else 0.0

        self._step_count += 1

        return ActionChunk(joints=joints, ios=ios, dt_ms=dt_ms)

    def _generate_joints(self, t: float, phase_offset: float) -> list[float]:
        """Generate sinusoidal joint positions."""
        result: list[float] = []
        for i in range(self.num_joints):
            # Each joint oscillates with slightly different phase
            joint_phase = i * 0.3 + phase_offset
            value = self.home_joints[i] + self.amplitude * math.sin(
                2.0 * math.pi * self.frequency * t + joint_phase
            )
            result.append(value)
        return result
