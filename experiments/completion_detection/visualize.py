"""Render recorded runs into a single self-contained HTML report.

Per run and per observed stream rate the report shows a timeline:

* every received frame as a tick, colored by wire-level category
  (TrajectoryEnded ticks are full-height — if you can't see one, the stream
  never delivered the terminal edge),
* the standstill flag as a green background band,
* the trajectory ``location`` as a progress line (a pause is a plateau),
* the state sequence the SDK's TrajectoryExecutionMachine would go through on
  exactly these frames, as a colored lane underneath,
* experiment events (start/pause/resume/…) as labelled markers.

Usage:
  PYTHONPATH=. uv run python -m experiments.completion_detection.visualize <batch-dir>...
"""

from __future__ import annotations

import html
import json
import sys
from pathlib import Path
from typing import Any

from experiments.completion_detection.analyze import analyze_run
from experiments.completion_detection.common import read_jsonl

CATEGORY_COLORS = {
    "no_execute": "#b3b3b3",
    "RUNNING": "#3b82f6",
    "END_OF_TRAJECTORY": "#dc2626",
    "PAUSED_BY_USER": "#f59e0b",
    "WAIT_FOR_IO": "#14b8a6",
    "PAUSED_ON_IO": "#8b5cf6",
    "execute_other": "#64748b",
}
FSM_COLORS = {
    "idle": "#d1d5db",
    "executing": "#3b82f6",
    "ending": "#f97316",
    "pausing": "#fbbf24",
    "paused": "#f59e0b",
    "ended": "#16a34a",
    "error": "#dc2626",
}
EVENT_MARKERS = {
    "start_sent": "start",
    "pause_sent": "pause",
    "resume_sent": "resume",
    "sdk_execute_started": "start",
    "sdk_execute_returned": "sdk returned",
    "sdk_execute_timeout": "sdk TIMEOUT",
    "finish_requested": "ws close req",
    "init_sent": "init",
}

WIDTH = 1100
MARGIN_LEFT = 8
PLOT_W = WIDTH - 2 * MARGIN_LEFT
FRAME_LANE_H = 46
FSM_LANE_H = 14
AXIS_H = 22
EVENT_H = 16


def _fmt(value: Any) -> str:
    return "–" if value is None else str(value)


def _stream_svg(
    frames: list[dict[str, Any]],
    fsm_transitions: list[dict[str, Any]],
    events: list[tuple[float, str]],
    t0_ns: int,
    t_min_s: float,
    t_max_s: float,
) -> str:
    span = max(t_max_s - t_min_s, 0.001)

    def x(t_s: float) -> float:
        return MARGIN_LEFT + (t_s - t_min_s) / span * PLOT_W

    height = EVENT_H + FRAME_LANE_H + FSM_LANE_H + AXIS_H + 10
    y_frames = EVENT_H
    y_fsm = EVENT_H + FRAME_LANE_H + 4
    parts = [
        f'<svg viewBox="0 0 {WIDTH} {height}" xmlns="http://www.w3.org/2000/svg" '
        f'style="width:100%;max-width:{WIDTH}px;background:#ffffff">'
    ]

    times = [(f["t_ns"] - t0_ns) / 1e9 for f in frames]

    # standstill background band
    for i, frame in enumerate(frames):
        if not frame.get("standstill"):
            continue
        x0 = x(times[i])
        x1 = x(times[i + 1]) if i + 1 < len(frames) else x0 + 1.5
        parts.append(
            f'<rect x="{x0:.1f}" y="{y_frames}" width="{max(x1 - x0, 1.0):.1f}" '
            f'height="{FRAME_LANE_H}" fill="#dcfce7"/>'
        )

    # location progress line (normalized to lane height)
    locations = [(t, f["location"]) for t, f in zip(times, frames) if f["location"] is not None]
    if locations:
        max_loc = max(loc for _, loc in locations) or 1.0
        points = " ".join(
            f"{x(t):.1f},{y_frames + FRAME_LANE_H - (loc / max_loc) * (FRAME_LANE_H - 6) - 3:.1f}"
            for t, loc in locations
        )
        parts.append(
            f'<polyline points="{points}" fill="none" stroke="#111827" '
            f'stroke-width="1" opacity="0.55"/>'
        )

    # frame ticks
    for t, frame in zip(times, frames):
        category = frame["category"]
        color = CATEGORY_COLORS.get(category, "#000")
        if category == "END_OF_TRAJECTORY":
            y0, y1, width_px = y_frames - 4, y_frames + FRAME_LANE_H + 4, 2.4
        elif category == "no_execute":
            y0, y1, width_px = y_frames + FRAME_LANE_H * 0.62, y_frames + FRAME_LANE_H, 1.0
        else:
            y0, y1, width_px = y_frames + FRAME_LANE_H * 0.18, y_frames + FRAME_LANE_H, 1.0
        parts.append(
            f'<line x1="{x(t):.1f}" y1="{y0:.1f}" x2="{x(t):.1f}" y2="{y1:.1f}" '
            f'stroke="{color}" stroke-width="{width_px}"/>'
        )

    # FSM lane
    for i, transition in enumerate(fsm_transitions):
        t_start = transition["t_s"]
        t_end = fsm_transitions[i + 1]["t_s"] if i + 1 < len(fsm_transitions) else t_max_s
        if t_start is None:
            continue
        state = transition["state"]
        color = FSM_COLORS.get(state, "#9ca3af")
        parts.append(
            f'<rect x="{x(t_start):.1f}" y="{y_fsm}" '
            f'width="{max(x(t_end if t_end is not None else t_max_s) - x(t_start), 1.0):.1f}" '
            f'height="{FSM_LANE_H}" fill="{color}"><title>{html.escape(state)}</title></rect>'
        )

    # event markers
    for t, label in events:
        if t < t_min_s or t > t_max_s:
            continue
        parts.append(
            f'<line x1="{x(t):.1f}" y1="{EVENT_H - 4}" x2="{x(t):.1f}" '
            f'y2="{y_fsm + FSM_LANE_H}" stroke="#111827" stroke-width="0.8" '
            f'stroke-dasharray="3,3" opacity="0.7"/>'
            f'<text x="{x(t) + 2:.1f}" y="{EVENT_H - 6}" font-size="9" '
            f'fill="#111827">{html.escape(label)}</text>'
        )

    # time axis
    y_axis = y_fsm + FSM_LANE_H + 14
    parts.append(
        f'<line x1="{MARGIN_LEFT}" y1="{y_axis - 10}" x2="{MARGIN_LEFT + PLOT_W}" '
        f'y2="{y_axis - 10}" stroke="#9ca3af" stroke-width="0.5"/>'
    )
    tick_step = max(round(span / 10), 1)
    tick = int(t_min_s)
    while tick <= t_max_s:
        if tick >= t_min_s:
            parts.append(
                f'<line x1="{x(tick):.1f}" y1="{y_axis - 13}" x2="{x(tick):.1f}" '
                f'y2="{y_axis - 7}" stroke="#9ca3af" stroke-width="0.8"/>'
                f'<text x="{x(tick):.1f}" y="{y_axis}" font-size="9" fill="#4b5563" '
                f'text-anchor="middle">{tick}s</text>'
            )
        tick += tick_step
    parts.append("</svg>")
    return "".join(parts)


def _legend() -> str:
    items = "".join(
        f'<span class="chip"><span class="swatch" style="background:{color}"></span>'
        f"{html.escape(name)}</span>"
        for name, color in CATEGORY_COLORS.items()
    )
    fsm_items = "".join(
        f'<span class="chip"><span class="swatch" style="background:{color}"></span>'
        f"{html.escape(name)}</span>"
        for name, color in FSM_COLORS.items()
    )
    return (
        f'<p class="legend"><b>Frames:</b> {items} '
        f'<span class="chip"><span class="swatch" style="background:#dcfce7"></span>'
        f"standstill band</span> — thin dark line = trajectory location</p>"
        f'<p class="legend"><b>FSM lane (SDK TrajectoryExecutionMachine replay):</b> {fsm_items}</p>'
    )


def _load_run_frames(run_dir: Path, rate: str) -> list[dict[str, Any]]:
    from experiments.completion_detection.analyze import _load_frames  # reuse parsing

    return _load_frames(run_dir / f"frames_{rate}.jsonl")


def render_batch(batch_dir: Path) -> Path:
    run_dirs = sorted(d for d in batch_dir.iterdir() if (d / "metadata.json").exists())
    if not run_dirs:
        raise SystemExit(f"no runs in {batch_dir}")

    overview_rows = []
    run_sections = []
    for run_dir in run_dirs:
        summary = analyze_run(run_dir)
        metadata = json.loads((run_dir / "metadata.json").read_text())
        events_raw = read_jsonl(run_dir / "events.jsonl")
        t0_ns = next(
            (e["t_ns"] for e in events_raw if e["event"] in ("start_sent", "sdk_execute_started")),
            None,
        )
        event_markers = [
            ((e["t_ns"] - t0_ns) / 1e9, EVENT_MARKERS[e["event"]])
            for e in events_raw
            if t0_ns and e["event"] in EVENT_MARKERS
        ]

        stream_blocks = []
        for rate, stream in summary["streams"].items():
            frames = _load_run_frames(run_dir, rate)
            if not frames or t0_ns is None:
                stream_blocks.append(f"<h4>{rate}</h4><p>no frames captured</p>")
                continue
            times = [(f["t_ns"] - t0_ns) / 1e9 for f in frames]
            t_min, t_max = min(times + [-0.5]), max(times)
            svg = _stream_svg(
                frames, stream["fsm"]["transitions"], event_markers, t0_ns, t_min, t_max
            )
            census = stream["census"]
            cats = ", ".join(f"{k}: {v}" for k, v in sorted(census["by_category"].items()))
            detectors = stream["detectors"]
            fsm = stream["fsm"]
            hang = ' <b class="bad">⚠ FSM WOULD HANG</b>' if fsm["would_hang"] else ""
            seq = census.get("sequence_number") or {}
            stream_blocks.append(
                f"<h4>stream @ {html.escape(rate)} — {census['total']} frames</h4>"
                f"{svg}"
                f'<p class="mono small">{html.escape(cats)} | standstill_true: '
                f"{census['standstill_true']} | seq gaps: {seq.get('gap_count', '–')} "
                f"(largest {seq.get('largest_gap', '–')})<br>"
                f"FSM final: <b>{html.escape(fsm['final_state'])}</b>{hang} | detectors: "
                f"edge_fsm={_fmt(detectors['edge_fsm_s'])}s, "
                f"arg3={_fmt(detectors['arg3_s'])}s ({detectors['arg3_path']}), "
                f"level={_fmt(detectors['level_s'])}s</p>"
            )
            ended_seen = census["by_category"].get("END_OF_TRAJECTORY", 0)
            overview_rows.append(
                f"<tr><td>{html.escape(summary['run'])}</td>"
                f"<td>{summary['mode']}</td><td>{summary['scenario']}</td>"
                f"<td>{html.escape(rate)}</td><td>{census['total']}</td>"
                f'<td class="{"ok" if ended_seen else "bad"}">{ended_seen}</td>'
                f'<td class="{"bad" if fsm["would_hang"] else "ok"}">'
                f"{html.escape(fsm['final_state'])}</td>"
                f"<td>{_fmt(detectors['edge_fsm_s'])}</td>"
                f"<td>{_fmt(detectors['arg3_s'])} ({detectors['arg3_path']})</td>"
                f"<td>{_fmt(detectors['level_s'])}</td></tr>"
            )

        sdk_line = ""
        if summary["mode"] == "sdk":
            returned = summary["sdk_execute_returned_s"]
            sdk_line = (
                "<p>sdk <code>execute()</code>: "
                + (
                    f"returned at <b>{returned}s</b>"
                    if returned is not None
                    else '<b class="bad">did not return (timeout)</b>'
                )
                + "</p>"
            )
        poll_line = ""
        if summary["poll"]:
            p = summary["poll"]
            poll_line = (
                f"<p>pull endpoint (GET state @ {metadata['poll_interval_s']}s): "
                f"{p['polls']} polls, execute present in {p['with_execute']}, "
                f"median latency {p['latency_ms_median']} ms</p>"
            )
        error_line = (
            f'<p class="bad">execution error: {html.escape(str(summary["execution_error"]))}</p>'
            if summary["execution_error"]
            else ""
        )
        run_sections.append(
            f"<section><h3>{html.escape(summary['run'])} — {summary['mode']} / "
            f"{summary['scenario']} (planned {summary['planned_duration_s']:.2f}s, "
            f"label {html.escape(summary['label'])})</h3>"
            f"{error_line}{sdk_line}{poll_line}{''.join(stream_blocks)}</section>"
        )

    title = f"Completion detection — {batch_dir.name}"
    doc = f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{html.escape(title)}</title>
<style>
 body {{ font: 14px/1.5 system-ui, sans-serif; color: #111827; background: #f9fafb;
        max-width: {WIDTH + 40}px; margin: 0 auto; padding: 16px; }}
 h2 {{ margin-top: 0; }}
 section {{ background: #fff; border: 1px solid #e5e7eb; border-radius: 8px;
            padding: 12px 16px; margin: 14px 0; }}
 table {{ border-collapse: collapse; font-size: 12.5px; width: 100%; background: #fff; }}
 th, td {{ border: 1px solid #e5e7eb; padding: 3px 8px; text-align: left; }}
 th {{ background: #f3f4f6; }}
 .ok {{ color: #16a34a; font-weight: 600; }}
 .bad {{ color: #dc2626; font-weight: 600; }}
 .mono {{ font-family: ui-monospace, monospace; }}
 .small {{ font-size: 12px; color: #374151; }}
 .legend {{ font-size: 12px; }}
 .chip {{ margin-right: 10px; white-space: nowrap; }}
 .swatch {{ display: inline-block; width: 10px; height: 10px; margin-right: 3px;
            border-radius: 2px; vertical-align: baseline; }}
</style></head><body>
<h2>{html.escape(title)}</h2>
<p>Received MotionGroupState frames per stream rate, the SDK state machine's replayed
state sequence, and three completion detectors compared. Companion to
<code>docs/architecture/incoming/execution-completion-detection.md</code>.
Times are seconds relative to the start request.</p>
{_legend()}
<table><thead><tr><th>run</th><th>mode</th><th>scenario</th><th>rate</th>
<th>frames</th><th>TrajectoryEnded frames</th><th>FSM final</th>
<th>edge_fsm (s)</th><th>arg3 (s)</th><th>level (s)</th></tr></thead>
<tbody>{"".join(overview_rows)}</tbody></table>
{"".join(run_sections)}
</body></html>"""
    out = batch_dir / "report.html"
    out.write_text(doc)
    return out


def main() -> None:
    targets = [Path(p) for p in sys.argv[1:]]
    if not targets:
        raise SystemExit(__doc__)
    for target in targets:
        out = render_batch(target)
        print(f"report: {out}")


if __name__ == "__main__":
    main()
