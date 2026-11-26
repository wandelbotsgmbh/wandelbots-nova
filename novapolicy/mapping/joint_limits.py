"""Keep commanded joint targets inside the robot's own limits (contract M7, ADR 0015).

The limits come from the motion group description's ``operation_limits.auto_limits.joints``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import logging
from typing import TYPE_CHECKING

from nova import api

if TYPE_CHECKING:
    from nova.cell.motion_group import MotionGroup

logger = logging.getLogger(__name__)

DEFAULT_MARGIN_RAD = 0.035
"""Inward margin, ~2 deg, for the robot overshooting its target; re-tune per robot (ADR 0015)."""

_RAD_TO_DEG = 57.29578


@dataclass
class JointLimits:
    """One motion group's position bounds, in radians, in its own axis order."""

    lower: tuple[float, ...]
    upper: tuple[float, ...]
    saturated: set[int] = field(default_factory=set)
    """Joints that have been clamped at least once, for one-time reporting."""

    def __len__(self) -> int:
        return len(self.lower)

    def clamp(self, step: list[float], motion_group_id: str = "") -> list[float]:
        """One target, held inside the bounds; reports each joint's first clip.

        A step whose width disagrees with the limits is returned untouched.
        """
        if len(step) != len(self.lower):
            return step
        held = list(step)
        for index, value in enumerate(step):
            low, high = self.lower[index], self.upper[index]
            if value < low:
                held[index] = low
            elif value > high:
                held[index] = high
            else:
                continue
            if index not in self.saturated:
                self.saturated.add(index)
                logger.warning(
                    "%s joint %d asked for %.1f deg, held at its limit [%.1f, %.1f] deg. "
                    "The policy is commanding outside this joint's travel.",
                    motion_group_id or "motion group",
                    index,
                    value * _RAD_TO_DEG,
                    low * _RAD_TO_DEG,
                    high * _RAD_TO_DEG,
                )
        return held


def limits_from_description(
    description: object, *, margin: float = DEFAULT_MARGIN_RAD
) -> JointLimits | None:
    """Read position bounds out of a motion group description; ``None`` when it has none."""
    operation_limits = getattr(description, "operation_limits", None)
    auto = getattr(operation_limits, "auto_limits", None)
    joints = getattr(auto, "joints", None)
    if not isinstance(joints, (list, tuple)) or not joints:
        return None
    lower: list[float] = []
    upper: list[float] = []
    for item in joints:
        position = getattr(item, "position", None)
        low = getattr(position, "lower_limit", None)
        high = getattr(position, "upper_limit", None)
        if low is None or high is None or high - low <= 2 * margin:
            # One unreported or degenerate bound makes the whole set unusable.
            return None
        lower.append(float(low) + margin)
        upper.append(float(high) - margin)
    return JointLimits(lower=tuple(lower), upper=tuple(upper))


async def fetch_joint_limits(
    motion_groups: list[MotionGroup], *, margin: float = DEFAULT_MARGIN_RAD
) -> dict[str, JointLimits]:
    """Position bounds per motion group id, skipping any that report none."""
    limits: dict[str, JointLimits] = {}
    for mg in motion_groups:
        try:
            description = await mg.get_description()
        except (
            OSError,
            RuntimeError,
            ValueError,
            TypeError,
            AttributeError,
            api.OpenApiException,  # an error response, e.g. 404 for the model
        ) as exc:
            # A failure to read limits sends unclamped; it never stops a run from starting.
            logger.warning("Could not read joint limits for %s: %s", mg.id, exc)
            continue
        found = limits_from_description(description, margin=margin)
        if found is None:
            logger.warning(
                "%s reports no joint limits; targets will be sent unclamped and an "
                "out-of-range one will pause the motion",
                mg.id,
            )
            continue
        limits[mg.id] = found
        logger.info(
            "%s joint limits: %s",
            mg.id,
            ", ".join(
                f"j{i}=[{low * _RAD_TO_DEG:.1f},{high * _RAD_TO_DEG:.1f}]"
                for i, (low, high) in enumerate(zip(found.lower, found.upper, strict=True))
            ),
        )
    return limits
