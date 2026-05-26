"""
Test: Circular motion via overlapping waypoint jogging chunks.

Verifies that the waypoint jogger correctly handles overlapping action chunks.
A mock policy generates a circular TCP motion by pre-computing joint targets
via inverse kinematics. Each policy call returns a 1s-lookahead joint chunk
starting from the CURRENT time — the server replaces any remaining portion of
the previous chunk with the new one (overlapping waypoints).

Uses PolicyExecutor with WaypointConfig so the Rerun viewer shows the execution.

A separate state stream tracks the actual TCP position throughout execution.
Both commanded and actual positions are saved to a parquet file for analysis.

Prerequisites:
    NOVA_API=http://172.31.11.129
    pip install pyarrow numpy

Run:
    NOVA_API=http://172.31.11.129 PYTHONPATH=. python policy/examples/test_circular_waypoint_jogging.py
"""

from __future__ import annotations

import asyncio
import contextlib
from dataclasses import dataclass, field
import math
import os
from pathlib import Path
import time

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq

import nova
from nova import run_program, viewers
from nova.types import Pose
from policy import ActionChunk, Observation, PolicyExecutor, PolicySchema, WaypointConfig

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

CONTROLLER_NAME = os.environ.get("CONTROLLER", "ur10e")
NOVA_API = os.environ.get("NOVA_API", "http://172.31.11.129")

# Circle parameters
CIRCLE_RADIUS_MM = float(os.environ.get("RADIUS", "50"))  # mm
CIRCLE_DURATION_S = float(os.environ.get("DURATION", "30"))  # active motion seconds
CIRCLE_REVOLUTIONS = int(os.environ.get("REVOLUTIONS", "3"))  # number of circles
CIRCLE_PAUSE_S = float(os.environ.get("CIRCLE_PAUSE_S", "2"))  # pause between circles
CIRCLE_PLANE = os.environ.get("PLANE", "xz")  # xz, xy, or yz

# Chunk parameters
CHUNK_DT_MS = float(os.environ.get("DT_MS", "60"))  # time between steps in the chunk
CHUNK_LOOKAHEAD_S = float(os.environ.get("LOOKAHEAD_S", "1.0"))  # each chunk covers this much future
CHUNK_MODE = os.environ.get("CHUNK_MODE", "overlap")  # overlap or append (non-overlap)
APPEND_EARLY_MS = int(float(os.environ.get("APPEND_EARLY_MS", "2000")))
APPEND_CHUNK_GAP_MS = int(float(os.environ.get("APPEND_CHUNK_GAP_MS", "0")))

# Fixed start position (away from singularities)
START_JOINTS = [-2.4227, -1.2388, -2.3509, -1.0777, -1.5469, 2.4399]

# Output
OUTPUT_DIR = Path(os.environ.get("OUTPUT_DIR", "policy/benchmarks/output"))


def wall_time_to_active_time(t: float) -> float:
    """Map wall-clock execution time to active trajectory time, including pauses."""
    if CHUNK_MODE == "append" and APPEND_CHUNK_GAP_MS > 0:
        chunk_duration_s = CHUNK_LOOKAHEAD_S
        segment_s = chunk_duration_s + APPEND_CHUNK_GAP_MS / 1000.0
        chunk_idx = int(max(t, 0.0) // segment_s)
        local_s = max(t, 0.0) - chunk_idx * segment_s
        return min(chunk_idx * chunk_duration_s + min(local_s, chunk_duration_s), CIRCLE_DURATION_S)

    if CIRCLE_PAUSE_S <= 0 or CIRCLE_REVOLUTIONS <= 1:
        return min(max(t, 0.0), CIRCLE_DURATION_S)

    circle_duration_s = CIRCLE_DURATION_S / CIRCLE_REVOLUTIONS
    segment_s = circle_duration_s + CIRCLE_PAUSE_S
    for circle_idx in range(CIRCLE_REVOLUTIONS):
        wall_start = circle_idx * segment_s
        active_start = circle_idx * circle_duration_s
        if t < wall_start + circle_duration_s:
            return min(active_start + max(t - wall_start, 0.0), CIRCLE_DURATION_S)
        if circle_idx < CIRCLE_REVOLUTIONS - 1 and t < wall_start + segment_s:
            return min(active_start + circle_duration_s, CIRCLE_DURATION_S)
    return CIRCLE_DURATION_S


def total_wall_duration_s() -> float:
    """Total wall time for the run including pauses."""
    if CHUNK_MODE == "append" and APPEND_CHUNK_GAP_MS > 0:
        n_chunks = math.ceil(CIRCLE_DURATION_S / CHUNK_LOOKAHEAD_S)
        return CIRCLE_DURATION_S + max(0, n_chunks - 1) * APPEND_CHUNK_GAP_MS / 1000.0
    return CIRCLE_DURATION_S + max(0, CIRCLE_REVOLUTIONS - 1) * max(0.0, CIRCLE_PAUSE_S)


# ---------------------------------------------------------------------------
# TCP position tracking (separate state stream)
# ---------------------------------------------------------------------------


@dataclass
class TrackingRecord:
    """Single tracking sample."""

    t: float  # elapsed time (s)
    tcp_x: float  # actual TCP position (mm)
    tcp_y: float
    tcp_z: float
    cmd_x: float  # commanded circle position (mm)
    cmd_y: float
    cmd_z: float
    joints: list[float] = field(default_factory=list)


class TcpTracker:
    """Tracks actual TCP position via a separate state stream."""

    def __init__(self, motion_group):
        self._mg = motion_group
        self._records: list[TrackingRecord] = []
        self._task: asyncio.Task | None = None
        self._running = False
        self._start_time: float = 0.0
        self._circle_center: tuple[float, float, float] | None = None

    @property
    def records(self) -> list[TrackingRecord]:
        return self._records

    def set_circle_center(self, center: tuple[float, float, float]):
        self._circle_center = center

    def set_start_time(self, t: float):
        self._start_time = t

    async def start(self):
        self._running = True
        self._start_time = time.monotonic()
        self._task = asyncio.create_task(self._stream_loop())

    async def stop(self):
        self._running = False
        if self._task:
            self._task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self._task
            self._task = None

    async def _stream_loop(self):
        """Stream state at high rate and record TCP positions."""
        try:
            stream = self._mg.stream_state(response_rate_msecs=10)
            async for state in stream:
                if not self._running:
                    break
                t = time.monotonic() - self._start_time
                tcp_pose = state.tcp_pose
                if tcp_pose is None:
                    continue

                # Compute where on the circle we expect to be at time t
                cmd_x, cmd_y, cmd_z = self._expected_position(t)

                self._records.append(
                    TrackingRecord(
                        t=t,
                        tcp_x=tcp_pose.position[0],
                        tcp_y=tcp_pose.position[1],
                        tcp_z=tcp_pose.position[2],
                        cmd_x=cmd_x,
                        cmd_y=cmd_y,
                        cmd_z=cmd_z,
                        joints=list(state.joint_position.root),
                    )
                )
        except asyncio.CancelledError:
            pass
        except Exception as e:
            print(f"[Tracker] Error: {e}")

    def _expected_position(self, t: float) -> tuple[float, float, float]:
        """Compute expected circle position at time t."""
        if self._circle_center is None:
            return (0.0, 0.0, 0.0)
        cx, cy, cz = self._circle_center
        active_t = wall_time_to_active_time(t)
        angular_rate = CIRCLE_REVOLUTIONS * 2 * math.pi / CIRCLE_DURATION_S
        angle = angular_rate * active_t
        if CIRCLE_PLANE == "xz":
            return (
                cx + CIRCLE_RADIUS_MM * math.cos(angle),
                cy,
                cz + CIRCLE_RADIUS_MM * math.sin(angle),
            )
        if CIRCLE_PLANE == "xy":
            return (
                cx + CIRCLE_RADIUS_MM * math.cos(angle),
                cy + CIRCLE_RADIUS_MM * math.sin(angle),
                cz,
            )
        # yz
        return (
            cx,
            cy + CIRCLE_RADIUS_MM * math.cos(angle),
            cz + CIRCLE_RADIUS_MM * math.sin(angle),
        )


# ---------------------------------------------------------------------------
# Precompute circle trajectory in joint space via IK
# ---------------------------------------------------------------------------


def generate_circle_poses(
    center: tuple[float, float, float],
    orientation: tuple[float, float, float],
    n_steps: int,
) -> list[Pose]:
    """Generate TCP poses along a circle (multiple revolutions)."""
    cx, cy, cz = center
    poses = []
    # Total angular travel = CIRCLE_REVOLUTIONS * 2π over CIRCLE_DURATION_S
    angular_rate = CIRCLE_REVOLUTIONS * 2 * math.pi / CIRCLE_DURATION_S
    for i in range(n_steps):
        t = i * (CHUNK_DT_MS / 1000.0)
        angle = angular_rate * t

        if CIRCLE_PLANE == "xz":
            x = cx + CIRCLE_RADIUS_MM * math.cos(angle)
            y = cy
            z = cz + CIRCLE_RADIUS_MM * math.sin(angle)
        elif CIRCLE_PLANE == "xy":
            x = cx + CIRCLE_RADIUS_MM * math.cos(angle)
            y = cy + CIRCLE_RADIUS_MM * math.sin(angle)
            z = cz
        else:  # yz
            x = cx
            y = cy + CIRCLE_RADIUS_MM * math.cos(angle)
            z = cz + CIRCLE_RADIUS_MM * math.sin(angle)

        poses.append(Pose(x, y, z, *orientation))
    return poses


async def compute_joint_trajectory(
    mg, tcp_name: str, poses: list[Pose], reference_joints: tuple[float, ...]
) -> list[list[float]]:
    """Convert TCP poses to joint targets via IK.

    Uses batch IK and picks the solution closest to the previous configuration
    to ensure smooth, continuous joint trajectories.
    """
    print(f"Computing IK for {len(poses)} poses...")

    # Batch IK — returns list[list[tuple[float, ...]]] (solutions per pose)
    all_solutions = await mg._inverse_kinematics(poses, tcp_name)

    joint_trajectory: list[list[float]] = []
    prev_joints = np.array(reference_joints)

    for i, solutions in enumerate(all_solutions):
        if not solutions:
            print(f"  WARNING: No IK solution for pose {i}, holding previous joints")
            joint_trajectory.append(list(prev_joints))
            continue

        # Shift each solution close to previous joints (2π wrapping)
        best_dist = float("inf")
        best_solution = list(prev_joints)
        for sol in solutions:
            shifted = np.array(sol)
            shifted += 2 * np.pi * np.round((prev_joints - shifted) / (2 * np.pi))
            dist = float(np.sum((shifted - prev_joints) ** 2))
            if dist < best_dist:
                best_dist = dist
                best_solution = list(shifted)

        prev_joints = np.array(best_solution)
        joint_trajectory.append(best_solution)

    print(f"IK complete: {len(joint_trajectory)} joint configurations computed")

    # Verify continuity
    max_step = 0.0
    for i in range(1, len(joint_trajectory)):
        step = max(
            abs(joint_trajectory[i][j] - joint_trajectory[i - 1][j])
            for j in range(len(joint_trajectory[0]))
        )
        max_step = max(max_step, step)
    print(f"Max joint step between consecutive poses: {math.degrees(max_step):.3f} deg")

    return joint_trajectory


# ---------------------------------------------------------------------------
# Save results to parquet
# ---------------------------------------------------------------------------


def save_tracking_to_parquet(records: list[TrackingRecord], output_path: Path):
    """Save tracking records to a parquet file."""
    if not records:
        print("No tracking records to save.")
        return

    table = pa.table(
        {
            "t": pa.array([r.t for r in records], type=pa.float64()),
            "tcp_x": pa.array([r.tcp_x for r in records], type=pa.float64()),
            "tcp_y": pa.array([r.tcp_y for r in records], type=pa.float64()),
            "tcp_z": pa.array([r.tcp_z for r in records], type=pa.float64()),
            "cmd_x": pa.array([r.cmd_x for r in records], type=pa.float64()),
            "cmd_y": pa.array([r.cmd_y for r in records], type=pa.float64()),
            "cmd_z": pa.array([r.cmd_z for r in records], type=pa.float64()),
        }
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    pq.write_table(table, output_path)
    print(f"Saved {len(records)} tracking records to {output_path}")


def save_commanded_trajectory(
    joint_trajectory: list[list[float]],
    poses: list[Pose],
    output_path: Path,
):
    """Save the precomputed joint trajectory + TCP poses as a parquet for replay."""
    n = len(joint_trajectory)
    dt_s = CHUNK_DT_MS / 1000.0

    table = pa.table(
        {
            "t": pa.array([i * dt_s for i in range(n)], type=pa.float64()),
            "tcp_x": pa.array([p.position[0] for p in poses[:n]], type=pa.float64()),
            "tcp_y": pa.array([p.position[1] for p in poses[:n]], type=pa.float64()),
            "tcp_z": pa.array([p.position[2] for p in poses[:n]], type=pa.float64()),
            **{
                f"joint_{j}": pa.array([traj[j] for traj in joint_trajectory], type=pa.float64())
                for j in range(len(joint_trajectory[0]))
            },
        }
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    pq.write_table(table, output_path)
    print(f"Saved commanded trajectory ({n} steps) to {output_path}")


def compute_tracking_error(records: list[TrackingRecord]) -> dict[str, float]:
    """Compute tracking error statistics."""
    if not records:
        return {}

    errors = []
    for r in records:
        err = math.sqrt(
            (r.tcp_x - r.cmd_x) ** 2 + (r.tcp_y - r.cmd_y) ** 2 + (r.tcp_z - r.cmd_z) ** 2
        )
        errors.append(err)

    return {
        "mean_error_mm": float(np.mean(errors)),
        "max_error_mm": float(np.max(errors)),
        "std_error_mm": float(np.std(errors)),
        "min_error_mm": float(np.min(errors)),
        "n_samples": len(errors),
    }


# ---------------------------------------------------------------------------
# Main program
# ---------------------------------------------------------------------------


@nova.program(
    id="test_circular_waypoint_jogging",
    name="Test Circular Waypoint Jogging",
    description=(
        "Tests overlapping waypoint jogging chunks with circular TCP motion. "
        "Sends 1s lookahead joint chunks via PolicyExecutor and tracks actual TCP."
    ),
    viewer=viewers.Rerun(),
)
async def test_circular_waypoint_jogging(ctx: nova.ProgramContext):
    cell = ctx.nova.cell()
    controller = await cell.controller(CONTROLLER_NAME)
    mg = controller[0]

    tcp_name = (await mg.tcp_names())[0]
    print(f"Using controller={CONTROLLER_NAME}, motion_group={mg.id}, tcp={tcp_name}")

    # --- Always move to fixed start position first ---
    from nova.actions import joint_ptp

    print("Moving to fixed start position via PTP...")
    actions = [joint_ptp(START_JOINTS)]
    trajectory = await mg.plan(actions, tcp_name)
    await mg.execute(trajectory, tcp_name, actions=actions)
    print("At start position.")

    # Get state at the fixed start position
    state = await mg.get_state()
    start_pose = state.pose
    start_joints = state.joints
    print(f"Start TCP pose: pos={start_pose.position}, orient={start_pose.orientation}")
    print(f"Start joints: {[f'{j:.4f}' for j in start_joints]}")

    # Circle center: offset so that current TCP position is ON the circle at angle=0
    # cos(0) = 1, so start_x = center_x + radius → center_x = start_x - radius
    if CIRCLE_PLANE == "xz":
        circle_center = (
            start_pose.position[0] - CIRCLE_RADIUS_MM,
            start_pose.position[1],
            start_pose.position[2],
        )
    elif CIRCLE_PLANE == "xy":
        circle_center = (
            start_pose.position[0] - CIRCLE_RADIUS_MM,
            start_pose.position[1],
            start_pose.position[2],
        )
    else:  # yz
        circle_center = (
            start_pose.position[0],
            start_pose.position[1] - CIRCLE_RADIUS_MM,
            start_pose.position[2],
        )

    print(f"Circle center: {circle_center}, radius: {CIRCLE_RADIUS_MM}mm, plane: {CIRCLE_PLANE}")
    execution_duration_s = total_wall_duration_s()
    print(
        f"Duration: {CIRCLE_DURATION_S}s active, revolutions: {CIRCLE_REVOLUTIONS}, "
        f"pause_between_circles: {CIRCLE_PAUSE_S}s, wall_duration: {execution_duration_s:.1f}s, "
        f"chunk_dt: {CHUNK_DT_MS}ms, lookahead: {CHUNK_LOOKAHEAD_S}s"
    )

    # --- Precompute joint trajectory via IK ---
    n_total_steps = int(CIRCLE_DURATION_S / (CHUNK_DT_MS / 1000.0))
    n_extra = int(CHUNK_LOOKAHEAD_S / (CHUNK_DT_MS / 1000.0))
    poses = generate_circle_poses(
        circle_center, tuple(start_pose.orientation), n_total_steps + n_extra
    )
    joint_trajectory = await compute_joint_trajectory(mg, tcp_name, poses, start_joints)

    # Verify IK continuity from start
    delta_start = max(abs(a - b) for a, b in zip(joint_trajectory[0], start_joints))
    print(f"Max joint delta at start: {math.degrees(delta_start):.3f} deg")

    n_chunk_steps = int(CHUNK_LOOKAHEAD_S / (CHUNK_DT_MS / 1000.0))
    print(f"\nTrajectory: {n_total_steps} steps total, {n_chunk_steps} steps per chunk")

    # --- Set up tracker ---
    tracker = TcpTracker(mg)
    tracker.set_circle_center(circle_center)

    # --- Define the policy ---
    # CHUNK_MODE=overlap: receding-horizon chunks at 20Hz.
    # CHUNK_MODE=append: non-overlapping chunks, sent APPEND_EARLY_MS before
    # their first timestamp. This keeps PolicyExecutor/Rerun while avoiding
    # overlapping waypoint intervals entirely.
    chunks_sent = 0
    skipped_policy_calls = 0
    steps_per_circle = int((CIRCLE_DURATION_S / CIRCLE_REVOLUTIONS) / (CHUNK_DT_MS / 1000.0))
    append_cursor = 0

    append_chunks: list[dict[str, int]] = []
    if CHUNK_MODE == "append":
        append_wall_cursor_ms = 0
        for circle_idx in range(CIRCLE_REVOLUTIONS):
            active_start_step = circle_idx * steps_per_circle
            active_end_step = min((circle_idx + 1) * steps_per_circle, n_total_steps)
            for step_idx in range(active_start_step, active_end_step, n_chunk_steps):
                chunk_end = min(step_idx + n_chunk_steps, active_end_step)
                start_time_ms = append_wall_cursor_ms
                append_chunks.append({
                    "send_at_ms": max(0, start_time_ms - APPEND_EARLY_MS),
                    "step_idx": step_idx,
                    "chunk_end": chunk_end,
                    "start_time_ms": start_time_ms,
                })
                chunk_duration_ms = int((chunk_end - step_idx) * CHUNK_DT_MS)
                append_wall_cursor_ms += chunk_duration_ms + APPEND_CHUNK_GAP_MS
        print(
            f"Non-overlap append mode: {len(append_chunks)} chunks, "
            f"append_early={APPEND_EARLY_MS}ms, chunk_gap={APPEND_CHUNK_GAP_MS}ms"
        )
    elif CHUNK_MODE != "overlap":
        raise ValueError(f"Unknown CHUNK_MODE={CHUNK_MODE!r}; expected overlap or append")

    # --- Policy ---
    # Timestamps use trajectory-absolute time (step_idx * dt).
    # The server syncs via jogger_session_timestamp_ms from the state stream.

    async def circle_policy(obs: dict) -> ActionChunk:
        nonlocal append_cursor, chunks_sent, skipped_policy_calls

        session = executor._sessions.get(mg.id)
        session_elapsed_ms = session.session_elapsed_ms if session is not None else 0
        t = session_elapsed_ms / 1000.0

        if CHUNK_MODE == "append":
            if session is not None and getattr(session, "_pending_request", None) is not None:
                skipped_policy_calls += 1
                return ActionChunk()
            if append_cursor >= len(append_chunks):
                skipped_policy_calls += 1
                return ActionChunk()
            spec = append_chunks[append_cursor]
            if session_elapsed_ms < spec["send_at_ms"]:
                skipped_policy_calls += 1
                return ActionChunk()
            append_cursor += 1
            step_idx = spec["step_idx"]
            chunk_end = spec["chunk_end"]
            start_time_ms = spec["start_time_ms"]
        else:
            # Index at wall-clock rate (no scaling)
            step_idx = int(session_elapsed_ms / CHUNK_DT_MS)
            step_idx = max(0, min(step_idx, n_total_steps - 1))
            chunk_end = min(step_idx + n_chunk_steps, len(joint_trajectory))
            # Trajectory-absolute timestamps (session scales dt internally)
            start_time_ms = int(step_idx * CHUNK_DT_MS)

        chunk = joint_trajectory[step_idx:chunk_end]

        # Pad only in overlap mode at the final end of the full trajectory.
        if CHUNK_MODE == "overlap" and (CIRCLE_PAUSE_S <= 0 or step_idx >= n_total_steps - n_chunk_steps):
            while len(chunk) < n_chunk_steps:
                chunk.append(chunk[-1] if chunk else joint_trajectory[-1])

        chunks_sent += 1
        if chunks_sent % 10 == 0 or chunks_sent == 1:
            print(
                f"  [policy] step={chunks_sent}, t={t:.2f}s, "
                f"traj_idx={step_idx}/{n_total_steps}, n={len(chunk)}, "
                f"ts={start_time_ms}..{start_time_ms + int((len(chunk) - 1) * CHUNK_DT_MS)}"
            )

        return ActionChunk(
            joints={mg.id: chunk},
            dt_ms=CHUNK_DT_MS,
            start_time_ms=start_time_ms,
        )

    # --- Schema: joint actions, TCP observation for tracking ---
    schema = PolicySchema(
        observations=[
            Observation.joint_positions("joints", source=mg),
            Observation.tcp("tcp_pose", source=mg, tcp=tcp_name, action=False),
        ],
    )

    # Use WaypointConfig for native server-side waypoint jogging.
    # policy_rate_hz=20 means the policy is called at 20Hz. Each new chunk
    # replaces the previous one mid-execution (overlapping).
    executor = PolicyExecutor(
        schema,
        circle_policy,
        timeout_s=execution_duration_s,
        policy_rate_hz=20,
        motion=WaypointConfig(state_rate_ms=10),
    )

    # --- Execute ---
    await tracker.start()
    print(f"\nExecuting circular motion for {execution_duration_s}s wall time...")
    print(f"Each policy call returns up to {n_chunk_steps} steps ({CHUNK_LOOKAHEAD_S}s lookahead)")
    if CHUNK_MODE == "append":
        print("Non-overlapping append chunks — no waypoint timestamp overlap.\n")
    else:
        print("New chunk overrides previous — server handles overlapping waypoints.\n")

    try:
        result = await executor.run()
        print(
            f"\nExecution complete: reason={result.reason}, "
            f"steps={result.steps}, duration={result.duration_s:.2f}s"
        )
        print(
            f"Policy called {chunks_sent} times "
            f"(~{chunks_sent / max(result.duration_s, 0.01):.1f} calls/s)"
        )
    except Exception as e:
        print(f"\nExecution error: {type(e).__name__}: {e}")
        import traceback

        traceback.print_exc()
    finally:
        await tracker.stop()

    # --- Analyze and save results ---
    records = tracker.records
    print(f"\nTracked {len(records)} TCP samples")

    if records:
        stats = compute_tracking_error(records)
        print("\n--- Tracking Error Statistics ---")
        print(f"  Mean error:  {stats['mean_error_mm']:.2f} mm")
        print(f"  Max error:   {stats['max_error_mm']:.2f} mm")
        print(f"  Std error:   {stats['std_error_mm']:.2f} mm")
        print(f"  Min error:   {stats['min_error_mm']:.2f} mm")
        print(f"  Samples:     {stats['n_samples']}")

        # Save tracking data
        output_path = OUTPUT_DIR / "circular_waypoint_jogging_tracking.parquet"
        save_tracking_to_parquet(records, output_path)

        # Save commanded trajectory (for replay tests)
        save_commanded_trajectory(
            joint_trajectory[:n_total_steps],
            poses[:n_total_steps],
            OUTPUT_DIR / "circular_waypoint_jogging_commanded.parquet",
        )


if __name__ == "__main__":
    run_program(test_circular_waypoint_jogging)
