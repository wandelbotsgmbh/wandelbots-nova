"""Relative action targets made absolute against the observed state (contract M5)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, cast

import numpy as np
from scipy.spatial.transform import Rotation

from novapolicy.mapping.poses import unwrap_tcp_rotations
from novapolicy.types import ActionChunk

if TYPE_CHECKING:
    from collections.abc import Iterable

_TCP_WIDTH = 6
"""Values of a NOVA TCP pose: [x, y, z, rx, ry, rz]."""


@dataclass(frozen=True, slots=True)
class TcpDelta:
    """How a relative TCP action's steps apply (see ``Action.tcp``)."""

    accumulate: bool = False
    """True: each step applies to the pose the previous step reached, not the observed one."""
    use_tool_coordinate_system: bool = False
    """True: offsets in the tool coordinate system at the step's start pose; False: in world."""


def apply_relative_mode(
    chunk: ActionChunk,
    states: dict[str, Any],
    relative_mgs: Iterable[str],
    tcp_deltas: dict[str, TcpDelta] | None = None,
) -> ActionChunk:
    """Convert relative action targets of ``relative_mgs`` to absolute ones (contract M5).

    TCP steps compose as ``tcp_deltas`` declares and are unwrapped onto the starting rotation.
    """
    relative_mgs = list(relative_mgs)
    if not relative_mgs:
        return chunk

    new_joints = dict(chunk.joints)
    new_tcp = dict(chunk.tcp)

    for mg_id in relative_mgs:
        state = states.get(mg_id)
        if state is None:
            continue

        # Joint deltas accumulate: step i is current + sum(deltas[0..i]).
        if mg_id in new_joints:
            running = list(state.joints)
            abs_steps = []
            for step in new_joints[mg_id]:
                running = [r + d for r, d in zip(running, step, strict=True)]
                abs_steps.append(list(running))
            new_joints[mg_id] = abs_steps

        if mg_id in new_tcp and hasattr(state, "pose") and state.pose is not None:
            start = [*state.pose.position, *state.pose.orientation]
            delta = (tcp_deltas or {}).get(mg_id, TcpDelta())
            new_tcp[mg_id] = compose_tcp_deltas(start, new_tcp[mg_id], delta)

    return ActionChunk(
        joints=new_joints,
        tcp=new_tcp,
        ios=chunk.ios,
        values=chunk.values,
        dt_ms=chunk.dt_ms,
        first_timestamp_ms=chunk.first_timestamp_ms,
        action_timestep=chunk.action_timestep,
    )


def compose_tcp_deltas(
    start: list[float], steps: list[list[float]], delta: TcpDelta
) -> list[list[float]]:
    """Absolute TCP targets ``[x, y, z, rx, ry, rz]`` (mm, rotation vector) from offsets.

    A rotation offset turns the tool about the TCP: pre-multiplied in world axes,
    post-multiplied in the tool coordinate system.
    """
    widths = sorted({len(step) for step in steps if len(step) != _TCP_WIDTH})
    if widths:
        msg = (
            f"Relative TCP offsets must be [x, y, z, rx, ry, rz] (6 values), got {widths}: "
            "declare the policy's rotation representation with orientation_type"
        )
        raise ValueError(msg)
    p0, r0 = np.asarray(start[:3], dtype=float), Rotation.from_rotvec(start[3:6])
    p, r = p0, r0
    out: list[list[float]] = []
    for step in steps:
        base_p, base_r = (p, r) if delta.accumulate else (p0, r0)
        dp, rv = np.asarray(step[:3], dtype=float), np.asarray(step[3:6], dtype=float)
        if delta.use_tool_coordinate_system:
            p = base_p + base_r.apply(dp)
            r = cast("Rotation", base_r * Rotation.from_rotvec(rv))
        else:
            p = base_p + dp
            r = cast("Rotation", Rotation.from_rotvec(rv) * base_r)
        out.append([*p.tolist(), *r.as_rotvec().tolist()])
    return unwrap_tcp_rotations(out, start[3:6])
