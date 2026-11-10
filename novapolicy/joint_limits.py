"""Keep commanded joint targets inside the robot's own limits.

NOVA enforces joint limits by **pausing** the motion: one out-of-range waypoint
ends the run with ``joint_limit (joints: [n])``. A policy that occasionally asks
for slightly more than a joint has is therefore fatal rather than merely
imperfect, and a residual policy asks for exactly that whenever the robot is not
quite where its reference expects.

The reference deployment stack for these policies clamps instead: it hands the
robot's real joint ranges to the policy core, which clips every target into them
before it is sent, so an impossible request becomes a saturated one and the
episode continues. This does the same thing on the robot side, where the limits
are actually known.

Clamping changes what the policy asked for, so it is reported rather than done
quietly: a joint that saturates is either genuinely out of distribution or
mis-mounted, and both are worth seeing.

The limits come from the motion group description's ``operation_limits``
``auto_limits.joints``, which is the only place NOVA exposes them. ``get_setup``
reports no position limits at all, and ``operation_limits`` has no
``joint_limits`` field despite the name suggesting one.
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
"""Inward margin, ~2 deg.

This has to absorb the robot *overshooting* its commanded target, not just
numerical rounding: the check that pauses the motion is on the measured
position, and a legal command is no help if the arm sails past it. Measured on a
manus elbow, a target held 0.5 deg inside the bound was overshot by 0.48 deg and
still tripped the limit by 0.02 deg. Two degrees leaves room for that with
margin to spare, at the cost of that much travel at each end.

Overshoot is a property of the controller and the approach speed, so expect to
re-tune this per robot; the symptom of it being too small is the very
``joint_limit`` pause clamping exists to prevent.
"""

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
        """One target, held inside the bounds. Reports each joint's first clip.

        A step whose width disagrees with the limits is returned untouched: the
        width is already checked against the policy's declared dof upstream, and
        guessing an alignment here would silently command the wrong joints.
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
    """Read position bounds out of a motion group description.

    ``None`` when the description carries none, which is a normal answer for
    robots whose limits NOVA does not report; the caller then sends unclamped,
    exactly as before.
    """
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
            # An unreported or degenerate bound cannot be honoured without
            # inventing travel; treat the whole set as unusable rather than
            # clamp some joints against guesses.
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
            # Any failure to read limits degrades to sending unclamped, which is
            # how this behaved before clamping existed. It must never be the
            # reason a run does not start.
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
