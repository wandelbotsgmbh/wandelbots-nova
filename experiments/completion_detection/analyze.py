"""Offline analysis of recorded experiment runs.

For every ``frames_<rate>.jsonl`` in a run directory this computes:

* a frame census (how often each wire-level category was observed — the §3
  measurement of the briefing, reproducible per run),
* a replay of the SDK's ``TrajectoryExecutionMachine`` over exactly the frames
  this stream delivered — i.e. what the SDK *would have concluded* on this
  stream ("associate received sequences with the FSM state sequence"),
* a comparison of three completion detectors on the same frames:
    - ``edge_fsm``      the shipped SDK machine (needs the TrajectoryEnded edge)
    - ``arg3_fallback``  what arg3-api does today (briefing §6b)
    - ``level``          standstill + no/terminal execute + at planned target +
                         dwell (the CiA-402-shaped detector, research §9)

Usage:
  PYTHONPATH=. uv run python -m experiments.completion_detection.analyze <run-or-batch-dir>...
"""

from __future__ import annotations

import json
import statistics
import sys
from pathlib import Path
from typing import Any

from experiments.completion_detection.common import (
    CAT_ENDED,
    CAT_NO_EXECUTE,
    CAT_PAUSED_IO,
    CAT_RUNNING,
    categorize_frame,
    frame_location,
    read_jsonl,
)
from nova import api
from nova.cell.movement_controller.trajectory_state_machine import TrajectoryExecutionMachine

AT_TARGET_TOLERANCE = 1e-2  # rad, mirrors branch 6a default
LEVEL_DWELL_S = 1.0
LEVEL_TERMINAL_CATEGORIES = {CAT_NO_EXECUTE, CAT_ENDED, CAT_PAUSED_IO}


def _load_frames(path: Path) -> list[dict[str, Any]]:
    frames = []
    for record in read_jsonl(path):
        state = record["state"]
        if "_unparseable" in state:
            continue
        frames.append(
            {
                "t_ns": record["t_ns"],
                "state": state,
                "category": categorize_frame(state),
                "standstill": bool(state.get("standstill")),
                "location": frame_location(state),
                "sequence_number": state.get("sequence_number"),
                "joints": state.get("joint_position"),
            }
        )
    return frames


def _census(frames: list[dict[str, Any]]) -> dict[str, Any]:
    counts: dict[str, int] = {}
    for frame in frames:
        counts[frame["category"]] = counts.get(frame["category"], 0) + 1
    deltas_ms = [
        (b["t_ns"] - a["t_ns"]) / 1e6 for a, b in zip(frames, frames[1:]) if b["t_ns"] > a["t_ns"]
    ]
    seq = [f["sequence_number"] for f in frames if f["sequence_number"] is not None]
    seq_gaps = [b - a for a, b in zip(seq, seq[1:]) if b > a + 1]
    return {
        "total": len(frames),
        "by_category": counts,
        "standstill_true": sum(1 for f in frames if f["standstill"]),
        "arrival_delta_ms": (
            {
                "median": round(statistics.median(deltas_ms), 2),
                "p95": round(sorted(deltas_ms)[int(len(deltas_ms) * 0.95)], 2),
                "max": round(max(deltas_ms), 2),
            }
            if deltas_ms
            else None
        ),
        "sequence_number": (
            {
                "first": seq[0],
                "last": seq[-1],
                "observed": len(seq),
                "gap_count": len(seq_gaps),
                "largest_gap": max(seq_gaps) if seq_gaps else 0,
            }
            if seq
            else None
        ),
    }


def _motion_window(frames: list[dict[str, Any]], t0_ns: int) -> tuple[int | None, int | None]:
    moving = [f["t_ns"] for f in frames if f["t_ns"] >= t0_ns and not f["standstill"]]
    return (moving[0], moving[-1]) if moving else (None, None)


def _replay_fsm(
    frames: list[dict[str, Any]], t0_ns: int, resume_times_ns: list[int]
) -> dict[str, Any]:
    """Feed the recorded frames through the SDK's state machine (as this branch ships it)."""
    machine = TrajectoryExecutionMachine()
    machine.send("start")
    transitions: list[dict[str, Any]] = [{"t_ns": t0_ns, "state": "executing"}]
    pending_resumes = sorted(resume_times_ns)
    t_ended_ns: int | None = None

    for frame in frames:
        if frame["t_ns"] < t0_ns:
            continue
        # A resume in the real system is an external `start` sent to the machine.
        while pending_resumes and pending_resumes[0] <= frame["t_ns"]:
            t_resume = pending_resumes.pop(0)
            if machine.is_paused:
                machine.send("start")
                transitions.append({"t_ns": t_resume, "state": "executing"})
        try:
            state = api.models.MotionGroupState.model_validate(frame["state"])
        except Exception as e:
            transitions.append({"t_ns": frame["t_ns"], "state": f"validate_error:{e}"})
            continue
        result = machine.process_motion_state(state)
        if result.state_changed:
            transitions.append({"t_ns": frame["t_ns"], "state": result.current_state_id})
            if machine.is_ended and t_ended_ns is None:
                t_ended_ns = frame["t_ns"]

    return {
        "final_state": machine.current_state.id,
        "t_ended_ns": t_ended_ns,
        "transitions": transitions,
    }


def _at_target(joints: list[float] | None, target: list[float] | None) -> bool:
    if not joints or not target or len(joints) != len(target):
        return False
    return max(abs(a - b) for a, b in zip(joints, target)) <= AT_TARGET_TOLERANCE


def _detectors(
    frames: list[dict[str, Any]], t0_ns: int, planned_final_joints: list[float] | None
) -> dict[str, Any]:
    movement_seen = False
    had_trajectory_execute = False
    arg3_primary_ns: int | None = None
    arg3_fallback_ns: int | None = None
    level_candidate_since_ns: int | None = None
    level_ns: int | None = None

    for frame in frames:
        if frame["t_ns"] < t0_ns:
            continue
        category, standstill = frame["category"], frame["standstill"]
        if not standstill or category == CAT_RUNNING:
            movement_seen = movement_seen or not standstill
        if category not in (CAT_NO_EXECUTE,):
            had_trajectory_execute = True

        # arg3-api (briefing §6b): primary = TrajectoryEnded && standstill;
        # fallback = execute cleared at standstill after movement was observed.
        if arg3_primary_ns is None and category == CAT_ENDED and standstill:
            arg3_primary_ns = frame["t_ns"]
        if (
            arg3_fallback_ns is None
            and movement_seen
            and had_trajectory_execute
            and category == CAT_NO_EXECUTE
            and standstill
        ):
            arg3_fallback_ns = frame["t_ns"]

        # level detector (research §9): standstill + terminal/no execute + at
        # planned target, continuously for LEVEL_DWELL_S. A user-pause never
        # fires it: PAUSED_BY_USER is not a terminal category and the robot is
        # not at the planned final joints.
        level_condition = (
            movement_seen
            and standstill
            and category in LEVEL_TERMINAL_CATEGORIES
            and _at_target(frame["joints"], planned_final_joints)
        )
        if level_condition:
            if level_candidate_since_ns is None:
                level_candidate_since_ns = frame["t_ns"]
            elif (
                level_ns is None and frame["t_ns"] - level_candidate_since_ns >= LEVEL_DWELL_S * 1e9
            ):
                level_ns = frame["t_ns"]
        else:
            level_candidate_since_ns = None

    return {
        "arg3_primary_ns": arg3_primary_ns,
        "arg3_fallback_ns": arg3_fallback_ns,
        "arg3_detected_ns": arg3_primary_ns or arg3_fallback_ns,
        "arg3_path": (
            "primary" if arg3_primary_ns else ("fallback" if arg3_fallback_ns else "none")
        ),
        "level_ns": level_ns,
    }


def _rel_s(t_ns: int | None, t0_ns: int | None) -> float | None:
    if t_ns is None or t0_ns is None:
        return None
    return round((t_ns - t0_ns) / 1e9, 3)


def analyze_run(run_dir: Path) -> dict[str, Any]:
    metadata = json.loads((run_dir / "metadata.json").read_text())
    events = read_jsonl(run_dir / "events.jsonl")
    event_times = {}
    for event in events:
        event_times.setdefault(event["event"], event["t_ns"])
    t0_ns = event_times.get("start_sent") or event_times.get("sdk_execute_started")
    resume_times = [e["t_ns"] for e in events if e["event"] == "resume_sent"]

    streams: dict[str, Any] = {}
    for frames_path in sorted(run_dir.glob("frames_*.jsonl")):
        rate = frames_path.stem.removeprefix("frames_")
        frames = _load_frames(frames_path)
        if not frames:
            streams[rate] = {"census": {"total": 0}, "note": "no frames captured"}
            continue
        t0 = t0_ns or frames[0]["t_ns"]
        t_move_start, t_move_end = _motion_window(frames, t0)
        replay = _replay_fsm(frames, t0, resume_times)
        detectors = _detectors(frames, t0, metadata.get("planned_final_joints"))
        streams[rate] = {
            "census": _census(frames),
            "motion_window_s": [_rel_s(t_move_start, t0), _rel_s(t_move_end, t0)],
            "fsm": {
                "final_state": replay["final_state"],
                "would_hang": replay["final_state"] not in ("ended", "paused"),
                "t_ended_s": _rel_s(replay["t_ended_ns"], t0),
                "transitions": [
                    {"t_s": _rel_s(t["t_ns"], t0), "state": t["state"]}
                    for t in replay["transitions"]
                ],
            },
            "detectors": {
                "edge_fsm_s": _rel_s(replay["t_ended_ns"], t0),
                "arg3_path": detectors["arg3_path"],
                "arg3_s": _rel_s(detectors["arg3_detected_ns"], t0),
                "level_s": _rel_s(detectors["level_ns"], t0),
                "latency_after_motion_end_s": {
                    "edge_fsm": _rel_s(replay["t_ended_ns"], t_move_end),
                    "arg3": _rel_s(detectors["arg3_detected_ns"], t_move_end),
                    "level": _rel_s(detectors["level_ns"], t_move_end),
                },
            },
        }

    # what the pull endpoint saw (research §10.5)
    poll_records = read_jsonl(run_dir / "poll.jsonl")
    poll_summary = None
    if poll_records:
        with_execute = sum(1 for r in poll_records if (r.get("state") or {}).get("execute"))
        latencies = [r["latency_ms"] for r in poll_records if "latency_ms" in r]
        poll_summary = {
            "polls": len(poll_records),
            "with_execute": with_execute,
            "errors": sum(1 for r in poll_records if "error" in r),
            "latency_ms_median": round(statistics.median(latencies), 1) if latencies else None,
        }

    summary = {
        "run": run_dir.name,
        "mode": metadata["mode"],
        "scenario": metadata["scenario"],
        "label": metadata["label"],
        "planned_duration_s": metadata["planned_duration_s"],
        "execution_error": metadata.get("execution_error"),
        "sdk_execute_returned_s": _rel_s(event_times.get("sdk_execute_returned"), t0_ns),
        "sdk_execute_timeout": "sdk_execute_timeout" in event_times,
        "streams": streams,
        "poll": poll_summary,
    }
    (run_dir / "summary.json").write_text(json.dumps(summary, indent=2))
    return summary


def _print_summary(summary: dict[str, Any]) -> None:
    print(f"\n=== {summary['run']}  ({summary['mode']}/{summary['scenario']})")
    if summary["execution_error"]:
        print(f"    EXECUTION ERROR: {summary['execution_error']}")
    if summary["mode"] == "sdk":
        returned = summary["sdk_execute_returned_s"]
        print(
            "    sdk execute(): "
            + (f"returned after {returned}s" if returned is not None else "DID NOT RETURN")
        )
    for rate, stream in summary["streams"].items():
        census = stream["census"]
        if not census["total"]:
            print(f"  [{rate}] no frames")
            continue
        cats = ", ".join(f"{k}:{v}" for k, v in sorted(census["by_category"].items()))
        fsm = stream["fsm"]
        detectors = stream["detectors"]
        print(f"  [{rate}] {census['total']} frames | {cats}")
        seq = census.get("sequence_number")
        if seq:
            print(
                f"        seq {seq['first']}→{seq['last']} observed {seq['observed']} "
                f"gaps {seq['gap_count']} (largest {seq['largest_gap']})"
            )
        print(
            f"        FSM: final={fsm['final_state']}"
            + (" ⚠ WOULD HANG" if fsm["would_hang"] else "")
            + f" | edge={detectors['edge_fsm_s']}s"
            f" arg3={detectors['arg3_s']}s({detectors['arg3_path']})"
            f" level={detectors['level_s']}s"
        )
    if summary["poll"]:
        p = summary["poll"]
        print(
            f"  [poll] {p['polls']} polls, execute present in {p['with_execute']}, "
            f"median latency {p['latency_ms_median']}ms, errors {p['errors']}"
        )


def main() -> None:
    targets = [Path(p) for p in sys.argv[1:]]
    if not targets:
        raise SystemExit(__doc__)
    run_dirs: list[Path] = []
    for target in targets:
        if (target / "metadata.json").exists():
            run_dirs.append(target)
        else:
            run_dirs.extend(sorted(d for d in target.iterdir() if (d / "metadata.json").exists()))
    if not run_dirs:
        raise SystemExit(f"no run directories found under {targets}")
    for run_dir in run_dirs:
        _print_summary(analyze_run(run_dir))


if __name__ == "__main__":
    main()
