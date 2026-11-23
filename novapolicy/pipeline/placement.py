"""Where a chunk sits on the session clock, and its trimming (contracts M6, T1, T2)."""

from __future__ import annotations

from dataclasses import dataclass
import logging
from typing import TYPE_CHECKING

from novapolicy.transport.clock import ServerMs
from novapolicy.types import ContinuousExecution

if TYPE_CHECKING:
    from novapolicy.types import ActionChunk, ExecutionMode

logger = logging.getLogger(__name__)


def _max_steps(chunk: ActionChunk) -> int:
    """Longest step sequence across all joint and TCP motion groups."""
    return max(
        (len(steps) for steps in (*chunk.joints.values(), *chunk.tcp.values())),
        default=0,
    )


def trim_chunk(chunk: ActionChunk, n: int) -> ActionChunk:
    """Trim an action chunk to its first ``n`` steps; ``n <= 0`` keeps all (contract M6).

    Warns when the policy returned fewer steps than ``n``.
    """
    if n <= 0:
        return chunk

    actual_steps = _max_steps(chunk)
    if 0 < actual_steps < n:
        logger.warning(
            "Policy returned %d steps but n_action_steps=%d. "
            "Check that n_action_steps <= policy action_horizon.",
            actual_steps,
            n,
        )

    trimmed_joints = (
        {mg_id: steps[:n] for mg_id, steps in chunk.joints.items()} if chunk.joints else {}
    )
    trimmed_tcp = {mg_id: steps[:n] for mg_id, steps in chunk.tcp.items()} if chunk.tcp else {}

    return chunk.model_copy(
        update={
            "joints": trimmed_joints,
            "tcp": trimmed_tcp,
            "values": {key: rows[:n] for key, rows in chunk.values.items()},
            "retained_prefix_steps": min(chunk.retained_prefix_steps, n),
        }
    )


def is_single_step(chunk: ActionChunk) -> bool:
    """Whether every motion group of ``chunk`` gets exactly one waypoint."""
    groups = [steps for steps in (*chunk.joints.values(), *chunk.tcp.values()) if steps]
    return bool(groups) and all(len(steps) == 1 for steps in groups)


@dataclass(frozen=True, slots=True)
class Placement:
    """Where a chunk's first step sits on the session clock: a timestamp, or ``None`` for now."""

    first_timestamp_ms: ServerMs | None
    lead: bool = False
    """Placed by the lead rule (contract T1)."""
    hold: bool = True
    """Whether ``HoldPadding`` may pad it: not a one-step chunk placed ahead in continuous
    execution (contract T6)."""


def placement(chunk: ActionChunk, *, execution: ExecutionMode) -> Placement:
    """Decide how a chunk is anchored on one session's timeline (contracts T1, T2).

    The lead rule, except: an explicit ``first_timestamp_ms`` is used verbatim, and in
    continuous execution a chunk with its own timestep, or several steps without a spacing,
    starts at "now" at the send.
    """
    if chunk.first_timestamp_ms >= 0:
        return Placement(ServerMs(chunk.first_timestamp_ms))
    if not isinstance(execution, ContinuousExecution):
        return Placement(None, lead=True)
    single = is_single_step(chunk)
    if chunk.action_timestep < 0 and (chunk.dt_ms > 0 or single):
        return Placement(None, lead=True, hold=not single)
    return Placement(None)
