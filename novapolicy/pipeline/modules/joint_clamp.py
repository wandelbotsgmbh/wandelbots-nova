"""``JointClamp``: hold every joint target inside the robot's limits (contract M7)."""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import TYPE_CHECKING

from novapolicy.mapping.joint_limits import DEFAULT_MARGIN_RAD

if TYPE_CHECKING:
    from novapolicy.pipeline.base import ModuleContext
    from novapolicy.transport.action_chunks import PendingChunk


@dataclass(frozen=True, slots=True)
class JointClamp:
    """Chunk module: hold every joint target ``margin_rad`` inside the robot's limits (M7).

    TCP waypoints pass unchanged; a robot that reports no limits is sent unclamped, with a
    warning.
    """

    margin_rad: float = DEFAULT_MARGIN_RAD

    def on_chunk(  # ruff: ignore[no-self-use] (the protocol's method)
        self, chunk: PendingChunk, context: ModuleContext
    ) -> PendingChunk:
        limits = context.joint_limits
        if limits is None:
            return chunk
        clamped = [limits.clamp(step, context.motion_group_id) for step in chunk.steps]
        return chunk if clamped == chunk.steps else replace(chunk, steps=clamped)
