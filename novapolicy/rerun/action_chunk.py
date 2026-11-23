"""Action chunk visualization: line strips, text logging."""

from __future__ import annotations

from functools import partial
from typing import TYPE_CHECKING

from novapolicy.rerun.constants import (
    BRIDGE_COLOR_END,
    BRIDGE_COLOR_START,
    BRIDGE_ENDPOINT_COLOR,
    BRIDGE_ENDPOINT_RADIUS_UI,
    BRIDGE_WIDTH_UI,
    CHUNK_COLOR_END,
    CHUNK_COLOR_START,
    CHUNK_TAIL_COLOR,
    CHUNK_TAIL_WIDTH_UI,
    CHUNK_WIDTH_UI,
    MIN_LINE_STEPS,
    MIN_TCP_COMPONENTS,
    lerp_color,
)
from novapolicy.rerun.context import log_points
from novapolicy.rerun.kinematics import joint_tcp_position
import rerun as rr

if TYPE_CHECKING:
    from collections.abc import Callable, Iterator

    from nova_rerun_bridge.dh_robot import DHRobot
    import numpy as np

    from novapolicy.rerun.context import RerunContext
    from novapolicy.types import ActionChunk

    Steps = list[list[float]]


def log_action_chunk(
    ctx: RerunContext, chunk: ActionChunk, step: int, *, n_action_steps: int = 0
) -> None:
    """Log each motion group's chunk as a TCP path, with its discarded tail, and as text.

    The first ``n_action_steps`` steps are the ones executed (all of them when
    0); they are drawn as an orange-to-yellow path, the rest as a gray tail.
    """
    ctx.set_time(step)
    for mg_id, suffix, steps, to_positions in _tcp_paths(ctx, chunk):
        split = n_action_steps if 0 < n_action_steps < len(steps) else len(steps)
        path = f"policy/{mg_id}/action_chunk{suffix}"
        executed = to_positions(steps[:split])
        # A single joint target is drawn as a marker; a single TCP target is not drawn.
        if suffix != "_tcp" or len(executed) >= MIN_LINE_STEPS:
            _log_line_strip(ctx, path, executed, width=CHUNK_WIDTH_UI)

        discarded = steps[split:]
        if not discarded:
            rr.log(f"{path}_tail", rr.Clear(recursive=False), recording=ctx.recording)
            continue
        tail = executed[-1:] + to_positions(discarded)
        if len(tail) >= MIN_LINE_STEPS:
            rr.log(
                f"{path}_tail",
                rr.LineStrips3D(
                    [tail],
                    colors=[CHUNK_TAIL_COLOR],
                    radii=rr.components.Radius.ui_points(CHUNK_TAIL_WIDTH_UI),
                ),
                recording=ctx.recording,
            )
    _log_text(ctx, chunk, step, n_action_steps=n_action_steps)


def log_bridge_chunk(ctx: RerunContext, chunk: ActionChunk, step: int) -> None:
    """Log an interpolated connector separately from orange policy output."""
    ctx.set_time(step)
    for mg_id, suffix, steps, to_positions in _tcp_paths(ctx, chunk):
        positions = to_positions(steps)
        if not positions:
            continue
        path = f"policy/{mg_id}/bridge_chunk{suffix}"
        _log_line_strip(
            ctx,
            path,
            positions,
            width=BRIDGE_WIDTH_UI,
            color_start=BRIDGE_COLOR_START,
            color_end=BRIDGE_COLOR_END,
        )
        rr.log(
            f"{path}_endpoint",
            rr.Points3D(
                [positions[-1]],
                colors=[BRIDGE_ENDPOINT_COLOR],
                radii=rr.components.Radius.ui_points(BRIDGE_ENDPOINT_RADIUS_UI),
            ),
            recording=ctx.recording,
        )

    rr.log(
        "policy/action_chunks",
        rr.TextLog(
            f"Step {step} | bridge | dt_ms={chunk.dt_ms} | "
            f"steps={{{', '.join(f'{group}: {len(steps)}' for group, steps in (*chunk.joints.items(), *chunk.tcp.items()))}}}",
            level=rr.TextLogLevel.TRACE,
        ),
        recording=ctx.recording,
    )


def _tcp_paths(
    ctx: RerunContext, chunk: ActionChunk
) -> Iterator[tuple[str, str, Steps, Callable[[Steps], Steps]]]:
    """Each motion group's steps, its entity suffix, and how to turn steps into TCP points.

    Joint targets go through forward kinematics to the TCP so the chunk lines
    up with the live TCP trail; a group without a kinematic model is skipped.
    TCP targets already are positions.
    """
    for mg_id, steps in chunk.joints.items():
        dh_robot = ctx.dh_robots.get(mg_id)
        if dh_robot is not None and steps:
            yield mg_id, "", steps, partial(_joint_positions, dh_robot, ctx.tcp_offsets.get(mg_id))
    for mg_id, steps in chunk.tcp.items():
        if steps:
            yield mg_id, "_tcp", steps, _tcp_positions


def _joint_positions(
    dh_robot: DHRobot, tcp_offset: np.ndarray | None, steps: Steps
) -> list[list[float]]:
    return [joint_tcp_position(dh_robot, joints, tcp_offset) for joints in steps]


def _tcp_positions(steps: Steps) -> list[list[float]]:
    return [step[:3] for step in steps if len(step) >= MIN_TCP_COMPONENTS]


def _log_text(ctx: RerunContext, chunk: ActionChunk, step: int, *, n_action_steps: int) -> None:
    """Log action chunk as inspectable text for offline review."""
    lines = [
        f"Step {step} | action_timestep={chunk.action_timestep} | dt_ms={chunk.dt_ms} | "
        f"first_timestamp_ms={chunk.first_timestamp_ms}"
    ]

    for mg_id, steps in chunk.joints.items():
        n_steps = len(steps)
        split = n_action_steps if 0 < n_action_steps < n_steps else n_steps
        lines.append(f"  {mg_id}: {n_steps} joint steps (execute {split})")
        if steps:
            joints_fmt = lambda j: "[" + ", ".join(f"{v:.4f}" for v in j) + "]"  # ruff: ignore[lambda-assignment]
            lines.append(f"    [0]   {joints_fmt(steps[0])}")
            if n_steps > MIN_LINE_STEPS:
                mid = n_steps // 2
                lines.append(f"    [{mid}] {joints_fmt(steps[mid])}")
            if n_steps > 1:
                lines.append(f"    [{n_steps - 1}] {joints_fmt(steps[-1])}")

    for mg_id, steps in chunk.tcp.items():
        n_steps = len(steps)
        lines.append(f"  {mg_id}: {n_steps} TCP steps")
        if steps:
            tcp_fmt = lambda s: "[" + ", ".join(f"{v:.2f}" for v in s) + "]"  # ruff: ignore[lambda-assignment]
            lines.append(f"    [0]   {tcp_fmt(steps[0])}")
            if n_steps > 1:
                lines.append(f"    [{n_steps - 1}] {tcp_fmt(steps[-1])}")

    if chunk.ios:
        for mg_id, ios in chunk.ios.items():
            lines.append(f"  {mg_id} IOs: {ios}")

    rr.log(
        "policy/action_chunks",
        rr.TextLog("\n".join(lines), level=rr.TextLogLevel.TRACE),
        recording=ctx.recording,
    )


def _log_line_strip(
    ctx: RerunContext,
    path: str,
    positions: list[list[float]],
    *,
    width: float,
    color_start: tuple[int, int, int] = CHUNK_COLOR_START,
    color_end: tuple[int, int, int] = CHUNK_COLOR_END,
) -> None:
    """Log a path shaded from ``color_start`` to ``color_end``, or a marker for one point."""
    n = len(positions)
    if n >= MIN_LINE_STEPS:
        colors = [lerp_color(color_start, color_end, i / max(n - 1, 1)) for i in range(n)]
        rr.log(
            path,
            rr.LineStrips3D(
                [positions], colors=colors, radii=rr.components.Radius.ui_points(width)
            ),
            recording=ctx.recording,
        )
    elif positions:
        log_points(ctx, path, positions, color_start)
