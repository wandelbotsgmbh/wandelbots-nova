"""Real-Time Chunking (RTC) wrapper for GR00T policy inference.

RTC reuses the tail of the previous action prediction as a warm start for the
diffusion denoising process, producing smoother overlapping action chunks.

This module provides ``RTCConfig`` and the logic used by ``Gr00tPolicyClient``
to compute RTC parameters per inference call. All state lives client-side so
RTC can be toggled at runtime without redeploying the server.

Requires the server-side patch that forwards ``options`` to the model::

    # gr00t/policy/gr00t_policy.py, line ~408:
    model_pred = self.model.get_action(**collated_inputs, options=options)

See ``policy/docs/RTC.md`` for the full investigation and design rationale.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import time


@dataclass
class RTCConfig:
    """Configuration for Real-Time Chunking.

    Parameters
    ----------
    denoising_steps:
        Number of diffusion denoising iterations on the server.
    max_overlap_factor:
        Maximum fraction of the action horizon that can overlap with
        the previous prediction. Range [0, 1]. Higher = more reuse.
    ramp_rate:
        Exponential ramp rate for blending between frozen and fully
        denoised steps. Higher = faster transition.
    latency_queue_size:
        Number of recent latency samples to average.
    systematic_latency_offset:
        Fixed latency offset in seconds added to measured inference time
        (accounts for ZMQ serialization, network, etc.).
    """

    denoising_steps: int = 8
    max_overlap_factor: float = 0.75
    ramp_rate: float = 3.0
    latency_queue_size: int = 10
    systematic_latency_offset: float = 0.02


@dataclass
class RTCState:
    """Mutable state for the RTC wrapper. Lives on the client (timing only).

    The previous action is stored on the server (GPU memory).
    The client only tracks timing to compute overlap/frozen steps.
    """

    action_horizon: int | None = None
    """Detected action horizon (number of timesteps per chunk)."""

    last_inference_time: float = 0.0
    """Monotonic time when the last inference completed."""

    last_overlap_steps: int = 0
    """Overlap steps from the most recent compute_rtc_options() call."""

    last_executed_steps: int = 0
    """Executed steps (motion committed since last inference) from the most
    recent compute_rtc_options() call."""

    latency_queue: deque = field(default_factory=lambda: deque(maxlen=10))
    """Recent inference latency samples in seconds."""

    def reset(self) -> None:
        """Clear RTC state (e.g., on task switch or episode boundary)."""
        self.last_inference_time = 0.0
        self.latency_queue.clear()


def compute_rtc_options(
    config: RTCConfig,
    state: RTCState,
    inference_latency: float,
    dt_ms: float,
) -> dict[str, object] | None:
    """Compute the RTC ``options`` dict to send to the server.

    Returns None if no previous action is available (first call).

    Parameters
    ----------
    config: RTC configuration.
    state: Mutable RTC state (timing info).
    inference_latency: Last measured inference round-trip in seconds.
    dt_ms: Step spacing of the action chunk in milliseconds.
        Used to derive the control frequency (steps/sec).
    """
    if state.action_horizon is None:
        return None

    control_freq = 1000.0 / dt_ms  # e.g. dt_ms=66.7 → 15 Hz

    # Update latency tracking
    total_latency = inference_latency + config.systematic_latency_offset
    max_chunk_time = (1.0 / control_freq) * state.action_horizon
    if total_latency < max_chunk_time:
        state.latency_queue.append(total_latency)

    avg_latency = sum(state.latency_queue) / len(state.latency_queue) if state.latency_queue else 0
    frozen_steps = int(avg_latency * control_freq)

    # Compute time since last inference → executed steps
    now = time.monotonic()
    between_time = now - state.last_inference_time if state.last_inference_time > 0 else 0
    executed_steps = int(control_freq * between_time)

    # Overlap steps: how much of the previous action to reuse
    max_rtc_steps = int(state.action_horizon * config.max_overlap_factor)
    overlap_steps = int(
        max(
            min(state.action_horizon - executed_steps + frozen_steps, max_rtc_steps),
            frozen_steps,
        )
    )

    # Clamp to valid range
    overlap_steps = max(0, min(overlap_steps, state.action_horizon))
    frozen_steps = max(0, min(frozen_steps, overlap_steps))

    # Remember for the executor's seam backdate. The robot is `executed_steps`
    # into the previous chunk; the new chunk reuses prev[H-overlap:] as its head,
    # so the robot's current position corresponds to new step
    # (executed_steps - (H - overlap)). Placing that step at "now" connects the
    # chunks, so the executor backdates the anchor by that many steps.
    state.last_overlap_steps = overlap_steps
    state.last_executed_steps = executed_steps

    return {
        "action_horizon": state.action_horizon,
        "rtc_overlap_steps": overlap_steps,
        "rtc_frozen_steps": frozen_steps,
        "rtc_ramp_rate": config.ramp_rate,
        "denoising_steps": config.denoising_steps,
    }


def detect_action_horizon(action: dict[str, object]) -> int | None:
    """Detect the action horizon from a server response.

    GR00T action arrays are ``(batch=1, time=T, dof)``; the horizon is the
    temporal axis ``T``, which is the second-to-last dimension. Using
    ``shape[-2]`` also handles an un-batched ``(T, dof)`` array. Returns the
    first such array's horizon, or ``None`` if no >=2-D array is present.
    """
    import numpy as np  # noqa: PLC0415

    for value in action.values():
        if isinstance(value, np.ndarray) and value.ndim >= 2:  # noqa: PLR2004
            return int(value.shape[-2])
    return None
