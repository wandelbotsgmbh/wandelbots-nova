"""Debug: Track exact timing and positions to find the snap-back desync.

Instruments every step of the execute-and-wait loop to verify:
1. Robot is truly stopped when we observe (chunk_done timing)
2. Position sent to inference matches robot's actual position
3. New chunk starts from the position we observed (not somewhere else)
4. Robot hasn't moved between observe and send

Run:
    NOVA_API=http://172.31.11.129 PYTHONPATH=. uv run python policy/examples/debug_snapback.py

Output: prints a timeline and writes /tmp/snapback_debug.jsonl for analysis.
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
import json
import math
import pathlib
import time
from typing import Any

import nova
from nova import api, run_program
from nova.actions import joint_ptp
from nova.cell import virtual_controller
from nova.program import ProgramPreconditions
from nova.types import MotionSettings
from policy import (
    ActionChunk,
    MotionConfig,
    Observation,
    PolicyExecutor,
    PolicySchema,
    WaypointConfig,
)
from policy.jogging.waypoint_session import is_waypoint_jogging_available

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

INFERENCE_DELAY_S = 0.8  # Simulates GR00T-like latency
CHUNK_SIZE = 16  # Steps per chunk (like GR00T horizon)
DT_MS = 66.7  # Step spacing (15 Hz)
TIMEOUT_S = 15.0
AMPLITUDE = 0.15  # Radians on joint 2
FREQUENCY = 0.3  # Hz
N_JOINTS = 6
OBS_PREFIX = "joints"  # schema key prefix → joints_1, joints_2, ...

HOME = (0.0, -1.571, 1.571, -1.571, -1.571, 0.0)
LOG_PATH = pathlib.Path("/tmp/snapback_debug.jsonl")  # noqa: S108


# ---------------------------------------------------------------------------
# Data recording
# ---------------------------------------------------------------------------


@dataclass
class StepRecord:
    """One inference cycle's complete timing + position record."""

    call: int
    t_observe: float  # when observation was read
    t_infer_done: float  # when inference completed (after sleep)
    t_chunk_ready: float  # when ActionChunk was built
    # Positions (joint 2 — the one we oscillate)
    obs_j2: float  # joint 2 from the flat observation
    chunk_first_j2: float  # first step of new chunk, joint 2
    chunk_last_j2: float  # last step of new chunk, joint 2
    prev_chunk_last_j2: float | None  # last step of PREVIOUS chunk
    # Gaps (should all be ~0 if properly synced)
    gap_obs_vs_prev_last: float | None  # obs_j2 - prev_chunk_last (robot finished?)
    gap_chunk_first_vs_obs: float  # chunk[0]_j2 - obs_j2 (chunk starts from obs?)
    # Live position at time chunk is ready (detects robot moved during inference)
    live_j2_at_send: float | None


_records: list[StepRecord] = []
_call_count = 0
_prev_chunk_last_j2: float | None = None
_t0 = 0.0
_mg_ref: Any = None  # MotionGroup for live state reads


# ---------------------------------------------------------------------------
# Mock slow policy (flat-feature interface)
# ---------------------------------------------------------------------------


async def mock_slow_policy(obs: dict[str, Any]) -> ActionChunk:
    """Simulates a slow policy that returns chunks based on observed position.

    Receives flat features: {joints_1: ..., joints_2: ..., joints_3: ..., ...}
    Returns ActionChunk with absolute joint positions.
    """
    global _call_count, _prev_chunk_last_j2

    t_observe = time.monotonic() - _t0
    _call_count += 1

    # Extract current joints from flat observation
    current_joints = [obs.get(f"{OBS_PREFIX}_{i+1}", HOME[i]) for i in range(N_JOINTS)]
    obs_j2 = current_joints[1]

    # --- Simulate slow inference ---
    await asyncio.sleep(INFERENCE_DELAY_S)
    t_infer_done = time.monotonic() - _t0

    # Read live robot position to check if it moved during inference
    live_j2 = None
    if _mg_ref is not None:
        try:
            live = await _mg_ref.get_state()
            live_j2 = live.joints[1]
        except Exception:
            pass

    # Generate chunk: sinusoidal motion on joint 2
    # Key: step[0] starts from OBSERVED position (like a real policy would).
    # Each subsequent step moves toward the desired trajectory.
    mg_id = "0@ur10e"
    steps = []
    for step_i in range(CHUNK_SIZE):
        # The target at each step is the ideal sinusoid at the FUTURE time
        step_time = t_observe + (step_i + 1) * (DT_MS / 1000.0)
        target = list(current_joints)  # copy observed as base
        target[1] = HOME[1] + AMPLITUDE * math.sin(2 * math.pi * FREQUENCY * step_time)
        steps.append(target)

    t_chunk_ready = time.monotonic() - _t0

    # Record
    gap_obs_prev = (obs_j2 - _prev_chunk_last_j2) if _prev_chunk_last_j2 is not None else None
    gap_c0_obs = steps[0][1] - obs_j2

    _records.append(StepRecord(
        call=_call_count,
        t_observe=round(t_observe, 4),
        t_infer_done=round(t_infer_done, 4),
        t_chunk_ready=round(t_chunk_ready, 4),
        obs_j2=round(obs_j2, 6),
        chunk_first_j2=round(steps[0][1], 6),
        chunk_last_j2=round(steps[-1][1], 6),
        prev_chunk_last_j2=round(_prev_chunk_last_j2, 6) if _prev_chunk_last_j2 is not None else None,
        gap_obs_vs_prev_last=round(gap_obs_prev, 6) if gap_obs_prev is not None else None,
        gap_chunk_first_vs_obs=round(gap_c0_obs, 6),
        live_j2_at_send=round(live_j2, 6) if live_j2 is not None else None,
    ))

    _prev_chunk_last_j2 = steps[-1][1]
    return ActionChunk(joints={mg_id: steps}, tcp={}, ios=None, dt_ms=DT_MS)


# ---------------------------------------------------------------------------
# Analysis
# ---------------------------------------------------------------------------


def print_analysis() -> None:
    if not _records:
        print("No records collected!")
        return

    deg = 57.2958
    print(f"\n{'=' * 100}")
    print("  SNAP-BACK DEBUG: Timing + Position Invariants (joint 2, degrees)")
    print(f"{'=' * 100}")
    print()
    print("  Invariants:")
    print("    gap1 = obs - prev_chunk_last  → Should be ~0 (robot finished moving)")
    print("    gap2 = live_at_send - obs     → Should be ~0 (robot didn't move during inference)")
    print("    gap3 = chunk[0] - obs         → Should be ~0 (chunk starts from observation)")
    print()
    print(f"  {'#':>3} {'t_obs':>5} {'t_done':>6} "
          f"{'obs°':>7} {'live°':>7} {'c[0]°':>7} {'c[-1]°':>7} │ "
          f"{'gap1°':>6} {'gap2°':>6} {'gap3°':>6}  flags")
    print(f"  {'-'*3} {'-'*5} {'-'*6} "
          f"{'-'*7} {'-'*7} {'-'*7} {'-'*7} {'─'} "
          f"{'-'*6} {'-'*6} {'-'*6}  {'-'*20}")

    issues_count = 0
    for r in _records:
        obs_d = r.obs_j2 * deg
        live_d = r.live_j2_at_send * deg if r.live_j2_at_send is not None else float("nan")
        c0_d = r.chunk_first_j2 * deg
        cn_d = r.chunk_last_j2 * deg

        gap1 = r.gap_obs_vs_prev_last * deg if r.gap_obs_vs_prev_last is not None else 0.0
        gap2 = (r.live_j2_at_send - r.obs_j2) * deg if r.live_j2_at_send is not None else 0.0
        gap3 = r.gap_chunk_first_vs_obs * deg

        flags = []
        if r.gap_obs_vs_prev_last is not None and abs(gap1) > 1.0:
            flags.append(f"ROBOT_NOT_DONE({gap1:+.1f}°)")
        if abs(gap2) > 0.5:
            flags.append(f"MOVED({gap2:+.1f}°)")
        if abs(gap3) > 0.5:
            flags.append(f"CHUNK_MISMATCH({gap3:+.1f}°)")
        if flags:
            issues_count += 1

        g1 = f"{gap1:+5.1f}°" if r.gap_obs_vs_prev_last is not None else "  n/a "
        g2 = f"{gap2:+5.1f}°" if r.live_j2_at_send is not None else "  n/a "
        g3 = f"{gap3:+5.1f}°"
        flag_str = " ← " + ", ".join(flags) if flags else ""

        print(
            f"  {r.call:>3} {r.t_observe:>5.1f} {r.t_infer_done:>6.1f} "
            f"{obs_d:>+6.1f}° {live_d:>+6.1f}° {c0_d:>+6.1f}° {cn_d:>+6.1f}° │ "
            f"{g1} {g2} {g3}{flag_str}"
        )

    print(f"\n  {'─' * 100}")
    print(f"  Total calls: {len(_records)}, Issues: {issues_count}")
    print(f"  Config: delay={INFERENCE_DELAY_S}s, chunk={CHUNK_SIZE}x{DT_MS}ms = "
          f"{CHUNK_SIZE * DT_MS / 1000:.2f}s")

    if issues_count == 0:
        print("\n  ✓ All invariants hold — no snap-back expected")
    else:
        print(f"\n  ⚠ {issues_count} desync events detected — snap-back likely!")

    LOG_PATH.write_text("\n".join(json.dumps(vars(r)) for r in _records) + "\n")
    print(f"\n  Log: {LOG_PATH}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


@nova.program(
    id="debug_snapback",
    name="Debug: Snap-back Timing Analysis",
    preconditions=ProgramPreconditions(
        controllers=[
            virtual_controller(
                name="ur10e",
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type="universalrobots-ur10e",
            ),
        ],
        cleanup_controllers=False,
    ),
)
async def debug_snapback(ctx: nova.ProgramContext):
    global _t0, _call_count, _prev_chunk_last_j2, _mg_ref
    _t0 = time.monotonic()
    _call_count = 0
    _prev_chunk_last_j2 = None
    _records.clear()

    cell = ctx.nova.cell()
    mg = (await cell.controller("ur10e"))[0]
    _mg_ref = mg

    # Home
    print("Moving to home...")
    fast = MotionSettings(tcp_velocity_limit=500.0)
    tcp = (await mg.tcp_names())[0]
    t = await mg.plan([joint_ptp(HOME, settings=fast)], tcp)
    await mg.execute(t, tcp, actions=[joint_ptp(HOME, settings=fast)])
    print("At home.\n")

    schema = PolicySchema(
        observations=[
            Observation.joint_positions(OBS_PREFIX, source=mg),
        ],
    )

    # Pick motion mode
    if is_waypoint_jogging_available():
        motion: MotionConfig | WaypointConfig = WaypointConfig()
        mode_name = "WaypointConfig (server-side)"
    else:
        motion = MotionConfig(velocity_limit=1.5, ramp_steps=3, p_gain=3.0)
        mode_name = "MotionConfig (client-side velocity profile)"
    print(f"Motion: {mode_name}")
    print(f"Running for {TIMEOUT_S}s with {INFERENCE_DELAY_S}s mock inference...\n")

    executor = PolicyExecutor(
        schema,
        mock_slow_policy,
        timeout_s=TIMEOUT_S,
        motion=motion,
    )

    try:
        result = await executor.run()
        print(f"\nDone: reason={result.reason} steps={result.steps} "
              f"duration={result.duration_s:.1f}s "
              f"({result.steps / result.duration_s:.2f} Hz)")
    except Exception as e:
        print(f"\nError: {type(e).__name__}: {e}")

    _mg_ref = None
    print_analysis()


if __name__ == "__main__":
    run_program(debug_snapback)
