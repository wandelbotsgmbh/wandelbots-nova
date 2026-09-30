"""How strictly an execution reads the controller's state stream.

:class:`ExecutionPolicy` bundles the knobs that decide what happens when the
state stream does something the SDK did not expect:

* ``strict`` — a frame that contradicts the tracked execution (``RUNNING`` while
  paused by user or ended, with no start issued) fails the execution with
  :class:`~nova.exceptions.UnexpectedTrajectoryState`. Without it the machine logs
  a warning and follows the controller.
* ``standstill`` — the :class:`StandstillConfig` that debounces the ``standstill``
  flag (see :mod:`nova.cell.movement_controller.standstill`).
* ``resume_detect_s`` / ``resume_window_s`` — supervision of a resume out of an IO
  pause that the controller does not take up (see ``move_forward``).

Presets: :meth:`ExecutionPolicy.robust` (the default), :meth:`ExecutionPolicy.strict`
(raw flag, contradictions fail — the rules before this policy existed, plus failing
on an ignored resume instead of waiting forever), :meth:`ExecutionPolicy.diagnose`
(robust decisions, but every jitter, contradiction and ignored resume fails, to
find out how often they happen).

The process-wide default comes from the environment and can be tuned per knob::

    NOVA_EXECUTION_POLICY=robust|strict|diagnose   (default: robust)
    NOVA_STANDSTILL_MOTION_VOTES=3
    NOVA_STANDSTILL_REST_VOTES=2
    NOVA_STANDSTILL_LOCATION_EPSILON=1e-6          ("none" disables)
    NOVA_STANDSTILL_JOINT_EPSILON=1e-3             ("none" disables)
    NOVA_RESUME_DETECT_MS=500                      ("none" disables)
    NOVA_RESUME_WINDOW_MS=1000
"""

from __future__ import annotations

import logging
import os
from dataclasses import dataclass, field, replace

from nova.cell.movement_controller.standstill import StandstillConfig

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class ExecutionPolicy:
    """Knobs for reading the state stream of one execution.

    Attributes:
        strict: Contradicting frames fail the execution instead of being followed.
        standstill: Debouncing of the ``standstill`` flag.
        resume_detect_s: After a resume start out of an IO pause, how long only the
            old ``PAUSED_ON_IO`` may be reported before the start counts as not taken
            up. ``None`` waits forever (no supervision).
        resume_window_s: Measured from the release edge of the pause signal: no
            start is sent later than this. A resume the controller has not taken up
            by then needs a new edge (signal back to pausing, then released again).
    """

    strict: bool = False
    standstill: StandstillConfig = field(default_factory=StandstillConfig.robust)
    resume_detect_s: float | None = 0.5
    resume_window_s: float = 1.0

    @classmethod
    def robust(cls) -> ExecutionPolicy:
        return cls()

    @classmethod
    def strict_policy(cls) -> ExecutionPolicy:
        return cls(strict=True, standstill=StandstillConfig.passthrough().with_strict(True))

    @classmethod
    def diagnose(cls) -> ExecutionPolicy:
        return cls(strict=True, standstill=StandstillConfig.robust().with_strict(True))

    @classmethod
    def from_env(cls, environ: dict[str, str] | None = None) -> ExecutionPolicy:
        env = os.environ if environ is None else environ
        name = env.get("NOVA_EXECUTION_POLICY", "robust").strip().lower()
        presets = {"robust": cls.robust, "strict": cls.strict_policy, "diagnose": cls.diagnose}
        if name not in presets:
            raise ValueError(
                f"NOVA_EXECUTION_POLICY={name!r}: expected one of {', '.join(sorted(presets))}"
            )
        policy = presets[name]()

        standstill = policy.standstill
        if "NOVA_STANDSTILL_MOTION_VOTES" in env:
            standstill = replace(standstill, motion_votes=int(env["NOVA_STANDSTILL_MOTION_VOTES"]))
        if "NOVA_STANDSTILL_REST_VOTES" in env:
            standstill = replace(standstill, rest_votes=int(env["NOVA_STANDSTILL_REST_VOTES"]))
        if "NOVA_STANDSTILL_LOCATION_EPSILON" in env:
            standstill = replace(
                standstill,
                location_epsilon=_optional_float(env["NOVA_STANDSTILL_LOCATION_EPSILON"]),
            )
        if "NOVA_STANDSTILL_JOINT_EPSILON" in env:
            standstill = replace(
                standstill, joint_epsilon=_optional_float(env["NOVA_STANDSTILL_JOINT_EPSILON"])
            )
        policy = replace(policy, standstill=standstill)

        if "NOVA_RESUME_DETECT_MS" in env:
            detect_ms = _optional_float(env["NOVA_RESUME_DETECT_MS"])
            policy = replace(
                policy, resume_detect_s=None if detect_ms is None else detect_ms / 1000.0
            )
        if "NOVA_RESUME_WINDOW_MS" in env:
            policy = replace(policy, resume_window_s=float(env["NOVA_RESUME_WINDOW_MS"]) / 1000.0)
        return policy


def _optional_float(value: str) -> float | None:
    value = value.strip().lower()
    if value in ("", "none", "off"):
        return None
    return float(value)


_default_policy: ExecutionPolicy | None = None


def default_execution_policy() -> ExecutionPolicy:
    """The process-wide policy used when an execution is given none.

    Read from the environment on first use (see the module docstring).
    """
    global _default_policy
    if _default_policy is None:
        _default_policy = ExecutionPolicy.from_env()
        logger.info("Execution policy: %s", _default_policy)
    return _default_policy


def set_default_execution_policy(policy: ExecutionPolicy | None) -> None:
    """Replace the process-wide default; ``None`` re-reads the environment on next use."""
    global _default_policy
    _default_policy = policy
