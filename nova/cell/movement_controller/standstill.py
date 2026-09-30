"""Standstill estimation on top of the controller's ``standstill`` flag.

``MotionGroupState.standstill`` is an *observed* property ("NOVA treats measured
joint velocities as 0"): a threshold on measured velocities. That threshold is
currently unreliable — the flag drops to ``false`` for single frames while the
robot is at rest — and every rule of :class:`TrajectoryExecutionMachine` that reads
it can then conclude too early or see motion that never happened (a flicker in
``armed`` concluded a user pause before the start was taken up and failed the
execution when ``RUNNING`` followed).

:class:`StandstillEstimator` turns the raw flag into a debounced decision,
``at_rest``, from three inputs:

* the raw flag itself, as consecutive votes (``motion_votes`` / ``rest_votes``);
* the **commanded** trajectory location (``execute.details.location``): it only
  advances while the controller executes a command, so a change between two
  frames of the same trajectory corroborates a ``standstill=false`` frame. It
  says nothing about a ``standstill=true`` frame: the first frame at the end of
  a trajectory has a new location and a settled robot;
* the **measured** joint positions (top-level ``joint_position``): a change above
  the joint's threshold between two frames corroborates ``standstill=false`` and
  vetoes a rest vote — measured joints that moved are not at rest. The unit is
  the joint's: rad for revolute joints (``joint_epsilon``), mm for prismatic ones
  (``prismatic_joint_epsilon``) — one threshold for both made encoder noise on a
  rail (1e-3 mm) count as motion. The estimator therefore needs the motion
  group's joint types; without them joint evidence is off.

Corroborated motion switches immediately; an uncorroborated ``standstill=false``
must repeat ``motion_votes`` times. Rest needs ``rest_votes`` consecutive standstill
frames whose measured joints did not move. :meth:`StandstillConfig.passthrough` (one vote each,
no corroboration) makes ``at_rest`` equal the raw flag on the same frame — the
behaviour before this estimator existed, for when upstream fixes the flag.

A *jitter* is a run of raw flags that contradicted the decision and ended before
it could change it. Each one is reported on the reading (``jitter``) so it can be
logged — or, with :attr:`StandstillConfig.strict`, fail the execution.
"""

from __future__ import annotations

import logging
from collections.abc import Sequence
from dataclasses import dataclass, replace

from nova import api

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class StandstillConfig:
    """Tuning of :class:`StandstillEstimator`.

    Attributes:
        motion_votes: Consecutive raw ``standstill=false`` frames that prove motion
            without corroboration. ``1`` trusts a single frame.
        rest_votes: Consecutive raw ``standstill=true`` frames (without motion
            evidence) that prove rest. ``1`` trusts a single frame.
        location_epsilon: Change of the commanded trajectory location between two
            frames of the same trajectory that proves motion; ``None`` disables it.
        joint_epsilon: Largest change of a measured **revolute** joint between two
            frames, in rad, that still counts as rest; a larger change proves
            motion. ``None`` disables it for revolute joints.
        prismatic_joint_epsilon: The same for **prismatic** joints, in mm.
            ``None`` disables it for prismatic joints.
        strict: A detected jitter fails the execution instead of being logged.
    """

    motion_votes: int = 1
    rest_votes: int = 1
    location_epsilon: float | None = None
    joint_epsilon: float | None = None
    prismatic_joint_epsilon: float | None = None
    strict: bool = False

    def __post_init__(self) -> None:
        if self.motion_votes < 1 or self.rest_votes < 1:
            raise ValueError("motion_votes and rest_votes must be at least 1")

    @classmethod
    def passthrough(cls) -> StandstillConfig:
        """``at_rest`` is the raw flag, on the same frame: no latency, no filtering."""
        return cls()

    @classmethod
    def robust(cls) -> StandstillConfig:
        """Debounced defaults for controllers whose flag flickers at rest."""
        return cls(
            motion_votes=3,
            rest_votes=2,
            location_epsilon=1e-6,
            joint_epsilon=1e-3,
            prismatic_joint_epsilon=0.1,
        )

    @property
    def is_passthrough(self) -> bool:
        return (
            self.motion_votes == 1
            and self.rest_votes == 1
            and self.location_epsilon is None
            and self.joint_epsilon is None
            and self.prismatic_joint_epsilon is None
        )

    def with_strict(self, strict: bool) -> StandstillConfig:
        return replace(self, strict=strict)


@dataclass(frozen=True)
class StandstillReading:
    """The estimator's view of one frame.

    Attributes:
        raw: The frame's ``standstill`` flag.
        at_rest: The debounced decision after this frame.
        evidence: What corroborated the frame as motion (``"location"``,
            ``"joints"``), or ``None`` when only the raw flag spoke.
        jitter: This frame ended a run of raw flags that contradicted the
            decision without changing it (``"motion"``: a ``standstill=false``
            run at rest, ``"rest"``: a ``standstill=true`` run while moving).
    """

    raw: bool
    at_rest: bool
    evidence: str | None = None
    jitter: str | None = None

    @property
    def disagrees(self) -> bool:
        return self.raw != self.at_rest


def _trajectory_details(state: api.models.MotionGroupState) -> api.models.TrajectoryDetails | None:
    if state.execute is not None and isinstance(
        state.execute.details, api.models.TrajectoryDetails
    ):
        return state.execute.details
    return None


def _joints(state: api.models.MotionGroupState) -> list[float] | None:
    joints = state.joint_position
    # ``Joints`` is a RootModel over a list; tests may hand in plain lists.
    values = getattr(joints, "root", joints)
    return list(values) if values is not None else None


class StandstillEstimator:
    """Debounces ``standstill`` with commanded-location and measured-joint evidence.

    Pure and synchronous: feed every frame of one motion group, in order, to
    :meth:`update`. One estimator follows the robot, not an operation, so it keeps
    its decision across starts and pauses.
    """

    def __init__(
        self, config: StandstillConfig | None = None, prismatic_joints: Sequence[bool] | None = None
    ) -> None:
        """Create an estimator.

        Args:
            config: Tuning; defaults to :meth:`StandstillConfig.passthrough`.
            prismatic_joints: Per joint, whether it is prismatic (mm) rather than
                revolute (rad), e.g. from the motion group's DH parameters. ``None``
                (unknown) disables joint evidence: a threshold in the wrong unit
                either never fires or fires on noise.
        """
        self.config = config or StandstillConfig.passthrough()
        self._joint_epsilons: list[float | None] | None = (
            None
            if prismatic_joints is None
            else [
                self.config.prismatic_joint_epsilon if prismatic else self.config.joint_epsilon
                for prismatic in prismatic_joints
            ]
        )
        self.at_rest: bool | None = None
        self._motion_streak = 0
        self._rest_streak = 0
        self._last_location: tuple[str, float] | None = None
        self._last_joints: list[float] | None = None

    def update(self, state: api.models.MotionGroupState) -> StandstillReading:
        config = self.config
        raw = state.standstill
        location_moved, joints_moved = self._evidence(state)
        # A standstill frame whose measured joints moved is not a rest vote.
        motion_frame = not raw or joints_moved
        evidence: str | None = None
        if motion_frame and joints_moved:
            evidence = "joints"
        elif motion_frame and location_moved:
            evidence = "location"

        if self.at_rest is None:
            # First frame: nothing to debounce against.
            self.at_rest = not motion_frame
            self._motion_streak = int(motion_frame)
            self._rest_streak = int(not motion_frame)
            return StandstillReading(raw=raw, at_rest=self.at_rest, evidence=evidence)

        jitter: str | None = None
        if motion_frame:
            if self._rest_streak and not self.at_rest:
                jitter = "rest"
            self._rest_streak = 0
            self._motion_streak += 1
        else:
            if self._motion_streak and self.at_rest:
                jitter = "motion"
            self._motion_streak = 0
            self._rest_streak += 1

        if self.at_rest:
            if motion_frame and (
                evidence is not None or self._motion_streak >= config.motion_votes
            ):
                self.at_rest = False
        elif self._rest_streak >= config.rest_votes:
            self.at_rest = True

        return StandstillReading(raw=raw, at_rest=self.at_rest, evidence=evidence, jitter=jitter)

    def _evidence(self, state: api.models.MotionGroupState) -> tuple[bool, bool]:
        """(commanded location moved, measured joints moved) since the previous frame."""
        config = self.config
        location_moved = False
        joints_moved = False

        details = _trajectory_details(state)
        if details is not None:
            location = (details.trajectory, details.location)
            last = self._last_location
            location_moved = (
                config.location_epsilon is not None
                and last is not None
                and last[0] == location[0]
                and abs(location[1] - last[1]) > config.location_epsilon
            )
            self._last_location = location

        joints = _joints(state)
        epsilons = self._joint_epsilons
        if joints is not None and epsilons is not None:
            last_joints = self._last_joints
            joints_moved = (
                last_joints is not None
                and len(last_joints) == len(joints) == len(epsilons)
                and any(
                    epsilon is not None and abs(now - before) > epsilon
                    for now, before, epsilon in zip(joints, last_joints, epsilons)
                )
            )
            self._last_joints = joints

        return location_moved, joints_moved
