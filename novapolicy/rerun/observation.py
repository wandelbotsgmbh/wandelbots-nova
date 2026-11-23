"""Measured robot state: robot mesh, joint scalars, TCP marker and trail."""

from __future__ import annotations

from typing import TYPE_CHECKING

from novapolicy.rerun.constants import TCP_TRAIL_COLOR
from novapolicy.rerun.context import log_points, log_trail
import rerun as rr

if TYPE_CHECKING:
    from nova.types import RobotState
    from novapolicy.rerun.context import RerunContext


def log_observation(ctx: RerunContext, states: dict[str, RobotState], step: int) -> None:
    """Log the states the policy observed at ``step``, without the TCP trail: the state
    streamer draws it from every state packet."""
    ctx.set_time(step)
    for mg_id, state in states.items():
        log_state(ctx, mg_id, state)


def log_state(
    ctx: RerunContext,
    mg_id: str,
    state: RobotState,
    trail: list[list[float]] | None = None,
) -> None:
    """Move the robot mesh to ``state`` and log its joints and TCP, extending ``trail``."""
    joints = list(state.joints)
    for i, value in enumerate(joints):
        rr.log(f"policy/{mg_id}/joints/j{i}", rr.Scalars(value), recording=ctx.recording)

    # One visualizer may hold several chains of a robot; this state drives only its own.
    visualizer = ctx.visualizers.get(mg_id)
    if visualizer is not None:
        visualizer.log_robot_geometry(joint_position={mg_id: joints})

    tcp = ctx.measured_tcp(mg_id, state)
    if tcp is None:
        return
    if trail is not None:
        log_trail(ctx, f"policy/{mg_id}/tcp_trail", trail, tcp, TCP_TRAIL_COLOR)
    log_points(ctx, f"policy/{mg_id}/tcp", [tcp], TCP_TRAIL_COLOR)
