"""Action chunk visualization: line strips, text logging."""

from __future__ import annotations

import time
from typing import TYPE_CHECKING, Any

from policy.rerun.constants import (
    _CHUNK_COLOR_END,
    _CHUNK_COLOR_START,
    _CHUNK_TAIL_COLOR,
    _CHUNK_TAIL_WIDTH_UI,
    _CHUNK_WIDTH_UI,
    _MIN_LINE_STEPS,
    _MIN_TCP_COMPONENTS,
    lerp_color,
)
import rerun as rr

if TYPE_CHECKING:
    from nova_rerun_bridge.dh_robot import DHRobot
    import numpy as np

    from policy.types import ActionChunk


def log_action_chunk(
    chunk: ActionChunk,
    step: int,
    *,
    start_time: float,
    dh_robots: dict[str, Any],
    tcp_offsets: dict[str, Any] | None = None,
    n_action_steps: int = 0,
) -> None:
    """Log action chunk as TCP path line strips and inspectable text."""
    _log_joint_chunk(
        chunk,
        step,
        start_time=start_time,
        dh_robots=dh_robots,
        tcp_offsets=tcp_offsets or {},
        n_action_steps=n_action_steps,
    )
    _log_tcp_chunk(chunk, step, start_time=start_time, n_action_steps=n_action_steps)
    _log_text(chunk, step, start_time=start_time, n_action_steps=n_action_steps)


def _log_joint_chunk(
    chunk: ActionChunk,
    step: int,
    *,
    start_time: float,
    dh_robots: dict[str, Any],
    tcp_offsets: dict[str, Any],
    n_action_steps: int,
) -> None:
    elapsed = time.monotonic() - start_time
    rr.set_time("policy_time", duration=elapsed)
    rr.set_time("policy_step", sequence=step)

    for mg_id, steps in chunk.joints.items():
        dh_robot = dh_robots.get(mg_id)
        if dh_robot is None or not steps:
            continue

        # Split into executed and discarded portions
        split = n_action_steps if 0 < n_action_steps < len(steps) else len(steps)
        executed_steps = steps[:split]
        discarded_steps = steps[split:]

        # Project each joint target to the TCP so the chunk lines up with the
        # live TCP trail. With the configured flange->TCP offset we apply it in
        # the flange frame; without it we fall back to the bare flange point.
        tcp_offset = tcp_offsets.get(mg_id)
        executed_positions = [_step_to_tcp(dh_robot, j, tcp_offset) for j in executed_steps]

        # Log executed portion with orange→yellow gradient
        _log_line_strip(
            f"policy/{mg_id}/action_chunk",
            executed_positions,
            gradient=True,
            width=_CHUNK_WIDTH_UI,
        )

        # Log discarded tail in dim gray
        _log_discarded_tail(
            f"policy/{mg_id}/action_chunk_tail",
            discarded_steps,
            dh_robot,
            executed_positions,
            tcp_offset,
        )


def _step_to_tcp(
    dh_robot: DHRobot, joint_target: list[float], tcp_offset: np.ndarray | None
) -> list[float]:
    """TCP position for a joint target: flange FK, then the TCP offset if known."""
    if tcp_offset is None:
        return dh_robot.calculate_joint_positions(joint_target)[-1]
    from policy.rerun.kinematics import flange_matrix  # noqa: PLC0415

    flange = flange_matrix(dh_robot, joint_target)
    return (flange @ tcp_offset)[:3, 3].tolist()


def _log_tcp_chunk(
    chunk: ActionChunk,
    step: int,
    *,
    start_time: float,
    n_action_steps: int,
) -> None:

    elapsed = time.monotonic() - start_time
    rr.set_time("policy_time", duration=elapsed)
    rr.set_time("policy_step", sequence=step)

    for mg_id, steps in chunk.tcp.items():
        if not steps:
            continue

        split = n_action_steps if 0 < n_action_steps < len(steps) else len(steps)
        executed = steps[:split]
        discarded = steps[split:]

        executed_positions = [[s[0], s[1], s[2]] for s in executed if len(s) >= _MIN_TCP_COMPONENTS]
        if len(executed_positions) >= _MIN_LINE_STEPS:
            n = len(executed_positions)
            colors = [
                lerp_color(_CHUNK_COLOR_START, _CHUNK_COLOR_END, i / max(n - 1, 1))
                for i in range(n)
            ]
            rr.log(
                f"policy/{mg_id}/action_chunk_tcp",
                rr.LineStrips3D(
                    [executed_positions],
                    colors=colors,
                    radii=rr.components.Radius.ui_points(_CHUNK_WIDTH_UI),
                ),
            )

        if discarded:
            tail_positions = [
                [s[0], s[1], s[2]] for s in discarded if len(s) >= _MIN_TCP_COMPONENTS
            ]
            if executed_positions:
                tail_positions = [executed_positions[-1], *tail_positions]
            if len(tail_positions) >= _MIN_LINE_STEPS:
                rr.log(
                    f"policy/{mg_id}/action_chunk_tcp_tail",
                    rr.LineStrips3D(
                        [tail_positions],
                        colors=[_CHUNK_TAIL_COLOR],
                        radii=rr.components.Radius.ui_points(_CHUNK_TAIL_WIDTH_UI),
                    ),
                )
        else:
            rr.log(f"policy/{mg_id}/action_chunk_tcp_tail", rr.Clear(recursive=False))


def _log_text(
    chunk: ActionChunk,
    step: int,
    *,
    start_time: float,
    n_action_steps: int,
) -> None:
    """Log action chunk as inspectable text for offline review."""

    elapsed = time.monotonic() - start_time
    rr.set_time("policy_time", duration=elapsed)
    rr.set_time("policy_step", sequence=step)

    lines = [f"Step {step} | dt_ms={chunk.dt_ms}"]

    for mg_id, steps in chunk.joints.items():
        n_steps = len(steps)
        split = n_action_steps if 0 < n_action_steps < n_steps else n_steps
        lines.append(f"  {mg_id}: {n_steps} joint steps (execute {split})")
        if steps:
            joints_fmt = lambda j: "[" + ", ".join(f"{v:.4f}" for v in j) + "]"  # noqa: E731
            lines.append(f"    [0]   {joints_fmt(steps[0])}")
            if n_steps > _MIN_LINE_STEPS:
                mid = n_steps // 2
                lines.append(f"    [{mid}] {joints_fmt(steps[mid])}")
            if n_steps > 1:
                lines.append(f"    [{n_steps - 1}] {joints_fmt(steps[-1])}")

    for mg_id, steps in chunk.tcp.items():
        n_steps = len(steps)
        lines.append(f"  {mg_id}: {n_steps} TCP steps")
        if steps:
            tcp_fmt = lambda s: "[" + ", ".join(f"{v:.2f}" for v in s) + "]"  # noqa: E731
            lines.append(f"    [0]   {tcp_fmt(steps[0])}")
            if n_steps > 1:
                lines.append(f"    [{n_steps - 1}] {tcp_fmt(steps[-1])}")

    if chunk.ios:
        for mg_id, ios in chunk.ios.items():
            lines.append(f"  {mg_id} IOs: {ios}")

    text = "\n".join(lines)
    rr.log(
        "policy/action_chunks",
        rr.TextLog(text, level=rr.TextLogLevel.TRACE),
    )


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _log_line_strip(
    entity_path: str,
    positions: list[list[float]],
    *,
    gradient: bool,
    width: float,
) -> None:
    """Log a line strip with gradient or uniform color."""

    if len(positions) >= _MIN_LINE_STEPS:
        n = len(positions)
        colors = (
            [lerp_color(_CHUNK_COLOR_START, _CHUNK_COLOR_END, i / max(n - 1, 1)) for i in range(n)]
            if gradient
            else [_CHUNK_COLOR_START] * n
        )
        rr.log(
            entity_path,
            rr.LineStrips3D(
                [positions],
                colors=colors,
                radii=rr.components.Radius.ui_points(width),
            ),
        )
    elif positions:
        rr.log(
            entity_path,
            rr.Points3D(
                positions,
                colors=[_CHUNK_COLOR_START],
                radii=rr.components.Radius.ui_points(4.0),
            ),
        )


def _log_discarded_tail(
    entity_path: str,
    discarded_steps: list[list[float]],
    dh_robot: DHRobot,
    bridge_from: list[list[float]],
    tcp_offset: np.ndarray | None = None,
) -> None:
    """Log discarded chunk tail in dim gray, connected from last executed point."""

    if not discarded_steps:
        rr.log(entity_path, rr.Clear(recursive=False))
        return

    tail_positions = [_step_to_tcp(dh_robot, jt, tcp_offset) for jt in discarded_steps]
    if bridge_from:
        tail_positions = [bridge_from[-1], *tail_positions]
    if len(tail_positions) >= _MIN_LINE_STEPS:
        rr.log(
            entity_path,
            rr.LineStrips3D(
                [tail_positions],
                colors=[_CHUNK_TAIL_COLOR],
                radii=rr.components.Radius.ui_points(_CHUNK_TAIL_WIDTH_UI),
            ),
        )
