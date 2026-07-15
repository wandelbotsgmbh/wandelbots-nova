"""Reconstruct a stable controller-timestamped path from a policy trajectory trace."""

from __future__ import annotations

import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import sys
from typing import Any

import numpy as np

_RETAINED_SEAM_STEPS = 4


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path, help="JSON trace written by PolicyExecutor")
    parser.add_argument("--motion-group", help="Motion group to analyze; defaults to the only one")
    parser.add_argument("--stable-path-output", type=Path, help="Optional reconstructed path JSON")
    return parser.parse_args()


def _percentiles(values: list[float]) -> str:
    if not values:
        return "n/a"
    p0, p50, p90, p100 = np.percentile(values, [0, 50, 90, 100])
    return f"min={p0:.1f} p50={p50:.1f} p90={p90:.1f} max={p100:.1f}"


def _initial_policy_boundary(
    trace: dict[str, Any],
    group_id: str,
) -> tuple[float, float, dict[int, list[float]]]:
    first_chunk = next(
        chunk
        for chunk in trace["policy_chunks"]
        if chunk["action_timestep"] == 0 and chunk["joints"].get(group_id)
    )
    initial_actions = first_chunk["joints"][group_id]
    first_target = np.asarray(initial_actions[0])
    first_request = trace["sessions"][group_id]["requests"][0]
    policy_start_index = min(
        range(len(first_request["steps"])),
        key=lambda index: np.linalg.norm(np.asarray(first_request["steps"][index]) - first_target),
    )
    if not np.allclose(first_request["steps"][policy_start_index], first_target):
        raise ValueError("Could not locate policy action zero in the initial bridge request")

    origin_ms = float(first_request["timestamps_ms"][policy_start_index])
    dt_ms = float(first_chunk["dt_ms"])
    actions = {
        int(first_chunk["action_timestep"]) + index: values
        for index, values in enumerate(initial_actions)
    }
    return origin_ms, dt_ms, actions


def analyze(  # noqa: C901
    trace: dict[str, Any],
    group_id: str,
) -> tuple[dict[str, Any], list[str]]:
    """Return the reconstructed stable path and a human-readable diagnosis."""
    origin_ms, dt_ms, published_actions = _initial_policy_boundary(trace, group_id)
    requests = trace["sessions"][group_id]["requests"]
    publications: dict[int, list[tuple[int, list[float]]]] = defaultdict(list)

    for request in requests[1:]:
        first_timestep = int(request["action_timestep"])
        if first_timestep < 0:
            continue
        for index, (timestamp_ms, joints) in enumerate(
            zip(request["timestamps_ms"], request["steps"], strict=True)
        ):
            timestep = first_timestep + index
            publications[timestep].append((int(timestamp_ms), joints))
            published_actions[timestep] = joints

    raw_predictions: dict[int, list[list[float]]] = defaultdict(list)
    policy_client_trace = trace.get("policy_client") or {}
    for chunk in policy_client_trace.get("raw_action_chunks", []):
        for action in chunk["actions"]:
            raw_predictions[int(action["timestep"])].append(action["values"])
    joint_count = len(next(iter(published_actions.values())))
    stable_actions = (
        {
            timestep: np.mean(values, axis=0)[:joint_count].tolist()
            for timestep, values in raw_predictions.items()
        }
        if raw_predictions
        else published_actions
    )

    actions = []
    for timestep in sorted(stable_actions):
        canonical_timestamp_ms = int(origin_ms + timestep * dt_ms)
        submitted = [timestamp for timestamp, _joints in publications.get(timestep, [])]
        actions.append(
            {
                "action_timestep": timestep,
                "timestamp_ms": canonical_timestamp_ms,
                "submitted_timestamps_ms": submitted,
                "joints": stable_actions[timestep],
            }
        )

    first_timestamp_errors = []
    canonical_ahead_at_send = []
    for request in requests[1:]:
        timestep = int(request["action_timestep"])
        if timestep < 0:
            continue
        canonical = origin_ms + timestep * dt_ms
        first_timestamp_errors.append(float(request["timestamps_ms"][0]) - canonical)
        canonical_ahead_at_send.append(canonical - float(request["server_sample_ms"]))

    retiming_ranges = [
        float(max(timestamps) - min(timestamps))
        for timestep in publications
        if len(timestamps := [value[0] for value in publications[timestep]]) > 1
    ]

    retained_position_errors_deg = []
    previous: dict[int, np.ndarray[Any, Any]] = {}
    for request in requests[1:]:
        first_timestep = int(request["action_timestep"])
        for index, joints in enumerate(request["steps"]):
            timestep = first_timestep + index
            values = np.asarray(joints)
            if index < _RETAINED_SEAM_STEPS and timestep in previous:
                retained_position_errors_deg.append(
                    float(np.max(np.abs(values - previous[timestep])) * 180.0 / math.pi)
                )
            previous[timestep] = values

    joint_path = np.asarray([action["joints"] for action in actions])
    derivative_lines = []
    for order in (1, 2, 3):
        differences = np.abs(np.diff(joint_path, n=order, axis=0)) * 180.0 / math.pi
        derivative_lines.append(
            f"stable joint difference {order}: p90={np.percentile(differences, 90):.2f}deg "
            f"max={np.max(differences):.2f}deg"
        )

    final_error = first_timestamp_errors[-1] if first_timestamp_errors else 0.0
    report = [
        f"motion group: {group_id}",
        f"controller boundary: action 0 = {origin_ms:.0f}ms",
        f"stable controller interval: {dt_ms:.6f}ms",
        (
            "stable path aggregation: global mean of raw inference chunks"
            if raw_predictions
            else "stable path aggregation: latest published action (raw chunks unavailable)"
        ),
        f"submitted first-waypoint error: {_percentiles(first_timestamp_errors)}",
        f"final submitted drift: {final_error:+.1f}ms",
        f"canonical first waypoint ahead at send: {_percentiles(canonical_ahead_at_send)}",
        f"same action retimed across requests: {_percentiles(retiming_ranges)}",
        (
            "retained overlap position error: "
            f"max={max(retained_position_errors_deg, default=0.0):.6f}deg"
        ),
        *derivative_lines,
    ]
    stable_path = {
        "format_version": 1,
        "source_trace": str(trace.get("source_trace", "")),
        "motion_group_id": group_id,
        "controller_origin_ms": origin_ms,
        "policy_dt_ms": dt_ms,
        "actions": actions,
    }
    return stable_path, report


def main() -> None:
    args = _parse_args()
    trace = json.loads(args.trace.read_text())
    group_ids = list(trace["sessions"])
    group_id = args.motion_group or (group_ids[0] if len(group_ids) == 1 else None)
    if group_id is None or group_id not in trace["sessions"]:
        raise SystemExit(f"Choose --motion-group from: {', '.join(group_ids)}")
    trace["source_trace"] = str(args.trace)
    stable_path, report = analyze(trace, group_id)
    sys.stdout.write("\n".join(report) + "\n")
    if args.stable_path_output is not None:
        args.stable_path_output.parent.mkdir(parents=True, exist_ok=True)
        args.stable_path_output.write_text(json.dumps(stable_path, indent=2))
        sys.stdout.write(f"stable path: {args.stable_path_output}\n")


if __name__ == "__main__":
    main()
