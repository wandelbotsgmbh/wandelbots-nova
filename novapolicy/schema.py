"""PolicySchema: declares what the policy observes and controls (docs/schema.md).

Writable observations (default) infer matching actions; an explicit ``Action`` is needed only
when the action key differs.
"""

from __future__ import annotations

from collections.abc import Awaitable, Callable
from dataclasses import dataclass, field
import logging
from types import SimpleNamespace
from typing import TYPE_CHECKING, Any, TypeVar, cast

import numpy as np

from nova.types import Pose
from novapolicy import poses
from novapolicy._sdk import get_controller_id
from novapolicy.chunking import TcpDelta
from novapolicy.joint_velocities import JointVelocities
from novapolicy.ops import apply_ops, apply_ops_inverse, reject_forward_only
from novapolicy.pose_sources import PoseSource
from novapolicy.types import ActionChunk

_TCP_SUFFIXES = ("x", "y", "z", "rx", "ry", "rz")
_QUATERNION_SUFFIXES = ("qx", "qy", "qz", "qw")
_EULER_SUFFIXES = ("e1", "e2", "e3")

if TYPE_CHECKING:
    from collections.abc import Iterator, Sequence

    from nova.cell.motion_group import MotionGroup
    from nova.types import RobotState
    from novapolicy.cameras import CameraSource
    from novapolicy.ops import ValueOp
    from novapolicy.types import ActionMode

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Value mappings
# ---------------------------------------------------------------------------


class Mapping:
    """Identity mapping between a hardware IO value and the policy's float."""

    def to_policy(  # ruff: ignore[no-self-use]
        self,
        hardware_value: bool | int | float,  # ruff: ignore[boolean-type-hint-positional-argument]
    ) -> float:
        if isinstance(hardware_value, bool):
            return 1.0 if hardware_value else 0.0
        return float(hardware_value)

    def to_hardware(  # ruff: ignore[no-self-use]
        self, policy_value: float
    ) -> bool | int | float | str:
        return policy_value


class BoolMapping(Mapping):
    """Map between hardware bool and policy float.

    Args:
        on: Policy value when hardware is True.
        off: Policy value when hardware is False.
        threshold: Values >= threshold map to True. Defaults to midpoint.
    """

    def __init__(self, on: float = 1.0, off: float = 0.0, threshold: float | None = None) -> None:
        self.on = on
        self.off = off
        self.threshold = threshold if threshold is not None else (on + off) / 2.0

    def to_policy(self, hardware_value: bool | int | float) -> float:  # ruff: ignore[boolean-type-hint-positional-argument]
        if isinstance(hardware_value, bool):
            return self.on if hardware_value else self.off
        return self.on if float(hardware_value) >= self.threshold else self.off

    def to_hardware(self, policy_value: float) -> bool:
        return policy_value >= self.threshold


# ---------------------------------------------------------------------------
# Observation entries (created via Observation factory)
# ---------------------------------------------------------------------------


@dataclass(slots=True)
class _ObsJoints:
    """Joint positions from one or more motion groups."""

    key: str
    source: MotionGroup | list[MotionGroup]
    action: bool = True
    mode: ActionMode = "absolute"
    ops: list[ValueOp] = field(default_factory=list)

    @property
    def sources(self) -> list[MotionGroup]:
        return (
            cast("list[MotionGroup]", self.source)
            if isinstance(self.source, list)
            else [self.source]
        )


@dataclass(slots=True)
class _ObsJointVelocities:
    """Joint velocities (rad/s) of one or more motion groups. Observation only."""

    key: str
    source: MotionGroup | list[MotionGroup]
    ops: list[ValueOp] = field(default_factory=list)

    @property
    def sources(self) -> list[MotionGroup]:
        return (
            cast("list[MotionGroup]", self.source)
            if isinstance(self.source, list)
            else [self.source]
        )


@dataclass(slots=True)
class _ObsTcp:
    """TCP pose from a motion group. Position in mm, orientation as rotation vector (rad)."""

    key: str
    source: MotionGroup
    tcp: str = ""
    action: bool = False
    mode: ActionMode = "absolute"
    position_ops: list[ValueOp] = field(default_factory=list)
    orientation_ops: list[ValueOp] = field(default_factory=list)
    orientation_type: poses.OrientationType | None = None  # None: the client's default
    coordinate_system: str = ""  # "": the controller's world


@dataclass(slots=True)
class _ObsPose:
    """The pose of an object, a target or a fixture: a NOVA ``Pose`` from a source."""

    key: str
    source: Pose | Callable[[], Awaitable[Pose]] | PoseSource
    position_ops: list[ValueOp] = field(default_factory=list)
    orientation_ops: list[ValueOp] = field(default_factory=list)
    orientation_type: poses.OrientationType | None = None  # None: rotation vector


@dataclass(slots=True)
class _ObsIO:
    """IO value (digital/analog). Writable by default."""

    key: str
    source: MotionGroup
    io: str
    mapping: Mapping = field(default_factory=Mapping)
    action: bool = True


@dataclass(slots=True)
class _ObsImage:
    """Camera image from a CameraSource."""

    key: str
    source: CameraSource
    max_age_s: float | None = None


@dataclass(slots=True)
class _ObsConstant:
    """Fixed value in every observation."""

    key: str
    value: Any


ComputedObsFn = Callable[[dict[str, Any]], Awaitable[dict[str, Any]]]


@dataclass(slots=True)
class _ObsComputed:
    """Async function called each step: ``async (obs_so_far) -> dict``."""

    fn: ComputedObsFn
    keys: tuple[str, ...] = ()


ObservationEntry = (
    _ObsJoints
    | _ObsJointVelocities
    | _ObsTcp
    | _ObsPose
    | _ObsIO
    | _ObsImage
    | _ObsConstant
    | _ObsComputed
)


class Observation:
    """Factory for observation entries."""

    @staticmethod
    def joint_positions(
        key: str,
        source: MotionGroup | list[MotionGroup],
        *,
        action: bool = True,
        mode: ActionMode = "absolute",
        ops: list[ValueOp] | None = None,
    ) -> _ObsJoints:
        """Observe joint positions. Writable by default (infers matching action).

        Args:
            key: Observation name the policy sees.
            source: One motion group, or several whose joints concatenate.
            action: Whether the policy also writes this channel.
            mode: ``"absolute"`` targets or ``"relative"`` deltas.
            ops: Unit conversions, inverted on the action (contract M1), e.g. ``[Rad2Deg()]``.
        """
        return _ObsJoints(key=key, source=source, action=action, mode=mode, ops=list(ops or []))

    @staticmethod
    def joint_velocities(
        key: str,
        source: MotionGroup | list[MotionGroup],
        *,
        ops: list[ValueOp] | None = None,
    ) -> _ObsJointVelocities:
        """Observe joint velocities in rad/s, as ``key_1 .. key_n`` (contract M9).

        Outside the executor, ``await schema.start_sources()`` first.

        Args:
            key: Observation name the policy sees.
            source: One motion group, or several whose joints concatenate.
            ops: Unit conversions applied to every value, e.g. ``ops=[Rad2Deg()]``.
        """
        return _ObsJointVelocities(key=key, source=source, ops=list(ops or []))

    @staticmethod
    def tcp(
        key: str,
        source: MotionGroup,
        *,
        tcp: str = "",
        action: bool = False,
        mode: ActionMode = "absolute",
        position_ops: list[ValueOp] | None = None,
        orientation_ops: list[ValueOp] | None = None,
        orientation_type: poses.OrientationType | None = None,
        coordinate_system: str = "",
    ) -> _ObsTcp:
        """Observe TCP pose [x, y, z, rx, ry, rz] in mm / rad (Nova native).

        ``action=True`` makes it a TCP action. ``position_ops`` and ``orientation_ops``
        convert the two parts apart; ``orientation_type`` gives a quaternion
        ``[qx, qy, qz, qw]`` or Euler angles instead of a rotation vector;
        ``coordinate_system`` (of the controller) replaces the world frame.
        """
        _check_orientation_ops(orientation_type, orientation_ops, f"Observation.tcp({key!r})")
        return _ObsTcp(
            key=key,
            source=source,
            tcp=tcp,
            action=action,
            mode=mode,
            position_ops=list(position_ops or []),
            orientation_ops=list(orientation_ops or []),
            orientation_type=orientation_type,
            coordinate_system=coordinate_system,
        )

    @staticmethod
    def pose(
        key: str,
        source: Pose | Callable[[], Awaitable[Pose]] | PoseSource,
        *,
        position_ops: list[ValueOp] | None = None,
        orientation_ops: list[ValueOp] | None = None,
        orientation_type: poses.OrientationType | None = None,
    ) -> _ObsPose:
        """Observe the pose of something that is not a motion group: an object, a target.

        ``source`` gives a NOVA ``Pose`` (mm, rotation vector): a fixed ``Pose``, an async
        callable returning one, or a :class:`PoseSource` read in the background. The policy
        sees it as ``Observation.tcp`` shows a TCP.
        """
        _check_orientation_ops(orientation_type, orientation_ops, f"Observation.pose({key!r})")
        return _ObsPose(
            key=key,
            source=source,
            position_ops=list(position_ops or []),
            orientation_ops=list(orientation_ops or []),
            orientation_type=orientation_type,
        )

    @staticmethod
    def io(
        key: str,
        source: MotionGroup,
        io: str,
        *,
        mapping: Mapping | None = None,
        action: bool = True,
    ) -> _ObsIO:
        """Observe an IO value. Writable by default (policy can write it back)."""
        return _ObsIO(key=key, source=source, io=io, mapping=mapping or Mapping(), action=action)

    @staticmethod
    def image(key: str, source: CameraSource, *, max_age_s: float | None = None) -> _ObsImage:
        """Observe a camera image. Source must have connect/read/disconnect.

        Args:
            key: Observation name the policy sees.
            source: Anything implementing ``CameraSource``.
            max_age_s: Oldest frame this channel may contribute, in seconds; defaults to the
                executor's ``camera_max_age_s``.
        """
        return _ObsImage(key=key, source=source, max_age_s=max_age_s)

    @staticmethod
    def constant(key: str, value: object) -> _ObsConstant:
        """Fixed value in every observation (e.g. language instruction)."""
        return _ObsConstant(key=key, value=value)

    @staticmethod
    def computed(fn: ComputedObsFn, *, keys: Sequence[str] = ()) -> _ObsComputed:
        """Async function called each step to add external data to the observation.

        ``fn`` is ``async (obs_so_far) -> dict``. ``keys`` names the keys it returns, so a
        client can check before the run that every model input is fed.
        """
        return _ObsComputed(fn=fn, keys=tuple(keys))


# ---------------------------------------------------------------------------
# Action entries (only needed when key differs from observation)
# ---------------------------------------------------------------------------


@dataclass(slots=True)
class _ActJoints:
    """Explicit joint position action."""

    key: str
    target: MotionGroup | list[MotionGroup]
    mode: ActionMode = "absolute"
    ops: list[ValueOp] = field(default_factory=list)

    @property
    def targets(self) -> list[MotionGroup]:
        return (
            cast("list[MotionGroup]", self.target)
            if isinstance(self.target, list)
            else [self.target]
        )


@dataclass(slots=True)
class _ActIO:
    """Explicit IO write action."""

    key: str
    target: MotionGroup
    io: str
    mapping: Mapping = field(default_factory=Mapping)


@dataclass(slots=True)
class _ActTcp:
    """Explicit TCP pose action."""

    key: str
    target: MotionGroup
    mode: ActionMode = "absolute"
    position_ops: list[ValueOp] = field(default_factory=list)
    orientation_ops: list[ValueOp] = field(default_factory=list)
    delta: TcpDelta = field(default_factory=TcpDelta)
    orientation_type: poses.OrientationType | None = None  # None: rotation vector
    coordinate_system: str = ""  # "": the controller's world


ComputedActFn = Callable[["ActionChunk"], Awaitable[None]]


@dataclass(slots=True)
class _ActComputed:
    """Async side-effect called with the policy's chunk: ``async (chunk) -> None``."""

    fn: ComputedActFn
    keys: tuple[str, ...] = ()


ActionEntry = _ActJoints | _ActTcp | _ActIO | _ActComputed

_FILL_ORDER = (
    _ObsJoints,
    _ObsJointVelocities,
    _ObsTcp,
    _ObsPose,
    _ObsIO,
    _ObsConstant,
    _ObsComputed,
)
"""The order ``build_observation`` fills observation kinds in."""

_Act = TypeVar("_Act", _ActJoints, _ActTcp, _ActIO)
_Obs = TypeVar("_Obs", _ObsJoints, _ObsTcp, _ObsIO)


def _groups(entry: ObservationEntry | ActionEntry) -> list[MotionGroup]:
    """The motion groups an observation reads or an action drives."""
    if isinstance(entry, (_ObsJoints, _ObsJointVelocities)):
        return entry.sources
    if isinstance(entry, _ActJoints):
        return entry.targets
    if isinstance(entry, (_ObsTcp, _ObsIO)):
        return [entry.source]
    if isinstance(entry, (_ActTcp, _ActIO)):
        return [entry.target]
    return []


class Action:
    """Factory for explicit action entries."""

    @staticmethod
    def joint_positions(
        key: str,
        target: MotionGroup | list[MotionGroup],
        *,
        mode: ActionMode = "absolute",
        ops: list[ValueOp] | None = None,
    ) -> _ActJoints:
        """Joint action with a key different from the observation; its ``ops`` win (M1)."""
        return _ActJoints(key=key, target=target, mode=mode, ops=list(ops or []))

    @staticmethod
    def tcp(
        key: str,
        target: MotionGroup,
        *,
        mode: ActionMode = "absolute",
        accumulate: bool = False,
        use_tool_coordinate_system: bool = False,
        position_ops: list[ValueOp] | None = None,
        orientation_ops: list[ValueOp] | None = None,
        orientation_type: poses.OrientationType | None = None,
        coordinate_system: str = "",
    ) -> _ActTcp:
        """TCP pose action, sent as Cartesian waypoints.

        ``mode="relative"``: offsets ``[dx, dy, dz, rx, ry, rz]`` composed with the pose at
        inference time (contract M5); ``accumulate=True`` applies each step to the pose the
        previous one reached; ``use_tool_coordinate_system=True`` takes the offsets in the
        tool frame. Rotations turn the tool about the TCP. The ops convert units before the
        offsets compose; ``orientation_type`` and ``coordinate_system`` as on
        ``Observation.tcp``.
        """
        _check_orientation_ops(orientation_type, orientation_ops, f"Action.tcp({key!r})")
        if mode != "relative" and (accumulate or use_tool_coordinate_system):
            msg = (
                "Action.tcp: accumulate and use_tool_coordinate_system apply to "
                "mode='relative' only"
            )
            raise ValueError(msg)
        return _ActTcp(
            key=key,
            target=target,
            mode=mode,
            position_ops=list(position_ops or []),
            orientation_ops=list(orientation_ops or []),
            delta=TcpDelta(
                accumulate=accumulate, use_tool_coordinate_system=use_tool_coordinate_system
            ),
            orientation_type=orientation_type,
            coordinate_system=coordinate_system,
        )

    @staticmethod
    def io(key: str, target: MotionGroup, io: str, *, mapping: Mapping | None = None) -> _ActIO:
        """IO write action."""
        return _ActIO(key=key, target=target, io=io, mapping=mapping or Mapping())

    @staticmethod
    def computed(fn: ComputedActFn, *, keys: Sequence[str] = ()) -> _ActComputed:
        """Async side effect run after each policy call, with the returned chunk.

        ``keys`` names action outputs the hook consumes that drive nothing in NOVA; every
        client puts them in ``chunk.values[key]``, one row per step.
        """
        return _ActComputed(fn=fn, keys=tuple(keys))


# ---------------------------------------------------------------------------
# PolicySchema
# ---------------------------------------------------------------------------


class PolicySchema:  # ruff: ignore[too-many-public-methods]
    """Declares what a policy observes and controls."""

    def __init__(
        self,
        observations: list[ObservationEntry],
        actions: list[ActionEntry] | None = None,
    ) -> None:
        self._observations = list(observations)
        self._actions = list(actions or [])
        # (controller id, coordinate system) -> its pose in world; load_coordinate_systems()
        self._frames: dict[tuple[str, str], np.ndarray] = {}
        velocity_groups = {
            mg.id: mg
            for o in self._observations
            if isinstance(o, _ObsJointVelocities)
            for mg in o.sources
        }
        self._joint_velocities = (
            JointVelocities(list(velocity_groups.values())) if velocity_groups else None
        )
        self._validate()

    def _validate(self) -> None:
        seen: set[str] = set()
        for o in self._observations:
            k: str | None = getattr(o, "key", None)
            if k is not None:
                if k in seen:
                    msg = f"Duplicate observation key: {k!r}"
                    raise ValueError(msg)
                seen.add(k)
        self._validate_ops()

        seen_act: set[str] = set()
        for a in self._actions:
            k = getattr(a, "key", None)
            if k is not None:
                if k in seen_act:
                    msg = f"Duplicate action key: {k!r}"
                    raise ValueError(msg)
                seen_act.add(k)

    def _validate_ops(self) -> None:
        """Refuse forward-only operators on a channel the policy writes."""
        for o in self._observations:
            if isinstance(o, _ObsJoints) and o.action:
                reject_forward_only(o.ops, f"Observation.joint_positions({o.key!r})")
            elif isinstance(o, _ObsTcp) and o.action:
                reject_forward_only(
                    [*o.position_ops, *o.orientation_ops], f"Observation.tcp({o.key!r})"
                )
        for a in self._actions:
            if isinstance(a, _ActJoints):
                reject_forward_only(a.ops, f"Action.joint_positions({a.key!r})")
            elif isinstance(a, _ActTcp):
                reject_forward_only([*a.position_ops, *a.orientation_ops], f"Action.tcp({a.key!r})")

    # -- Value operators on the action path --

    @property
    def _joint_action_ops(self) -> dict[str, list[ValueOp]]:
        """Motion group id -> joint operators to invert on the action path."""
        return {
            mg.id: entry.ops
            for entry in self._action_entries(_ActJoints, _ObsJoints)
            if entry.ops
            for mg in _groups(entry)
        }

    @property
    def _tcp_action_specs(self) -> dict[str, _ObsTcp | _ActTcp]:
        """Motion group id -> the entry that defines its TCP action's units and frame."""
        return {_groups(entry)[0].id: entry for entry in self._action_entries(_ActTcp, _ObsTcp)}

    def _action_entries(self, act_type: type[_Act], obs_type: type[_Obs]) -> list[_Act | _Obs]:
        """Explicit ``act_type`` actions, then writable ``obs_type`` observations of other keys."""
        explicit: list[_Act | _Obs] = [a for a in self._actions if isinstance(a, act_type)]
        keys = {entry.key for entry in explicit}
        return explicit + [
            o
            for o in self._observations
            if isinstance(o, obs_type) and o.action and o.key not in keys
        ]

    def tcp_action_width(self, motion_group_id: str) -> int:
        """Values per step of a motion group's TCP action: 3 position + 3 or 4 orientation."""
        entry = self._tcp_action_specs.get(motion_group_id)
        return 6 if entry is None else tcp_width(entry)

    # -- Coordinate systems --

    async def load_coordinate_systems(self) -> None:
        """Fetch the world pose of every declared coordinate system; the executor calls it."""
        entries: list[_ObsTcp | _ActTcp] = [o for o in self._observations if isinstance(o, _ObsTcp)]
        entries += [a for a in self._actions if isinstance(a, _ActTcp)]
        for entry in entries:
            if not entry.coordinate_system:
                continue
            mg = entry.source if isinstance(entry, _ObsTcp) else entry.target
            key = (get_controller_id(mg), entry.coordinate_system)
            if key not in self._frames:
                self._frames[key] = await poses.coordinate_system_in_world(
                    mg, entry.coordinate_system
                )

    async def start_sources(self) -> None:
        """Start the background observation sources; the executor calls it before a run."""
        if self._joint_velocities is not None:
            await self._joint_velocities.start()
        for source in self._background_pose_sources:
            await source.start()

    def observe_state(
        self, motion_group_id: str, joints: Sequence[float], sample_time_s: float
    ) -> None:
        """Feed one streamed state packet, stamped with its measurement time."""
        if self._joint_velocities is not None:
            self._joint_velocities.feed(motion_group_id, joints, sample_time_s)

    async def stop_sources(self) -> None:
        """Stop what ``start_sources`` started. Safe to call when nothing was started."""
        if self._joint_velocities is not None:
            await self._joint_velocities.stop()
        for source in self._background_pose_sources:
            await source.stop()

    @property
    def _background_pose_sources(self) -> list[PoseSource]:
        sources: dict[int, PoseSource] = {}
        for o in self._observations:
            if isinstance(o, _ObsPose) and isinstance(o.source, PoseSource):
                sources.setdefault(id(o.source), o.source)
        return list(sources.values())

    def frame(self, entry: _ObsTcp | _ActTcp) -> np.ndarray | None:
        """The pose in world of ``entry``'s coordinate system; None for the world."""
        if not entry.coordinate_system:
            return None
        mg = entry.source if isinstance(entry, _ObsTcp) else entry.target
        key = (get_controller_id(mg), entry.coordinate_system)
        if key not in self._frames:
            msg = (
                f"Coordinate system {entry.coordinate_system!r} is not loaded: "
                "await schema.load_coordinate_systems() first"
            )
            raise RuntimeError(msg)
        return self._frames[key]

    def tcp_observation_values(self, entry: _ObsTcp, state: RobotState) -> list[float]:
        """The observed TCP pose as the policy sees it (contract M3)."""
        pose = poses.world_to_frame(
            [*state.pose.position, *state.pose.orientation], self.frame(entry)
        )
        return _policy_pose_values(pose, entry)

    def policy_frame_states(self, states: dict[str, RobotState]) -> dict[str, Any]:
        """``states`` with each TCP action group's pose in its action's coordinate system."""
        result: dict[str, Any] = dict(states)
        for group_id, entry in self._tcp_action_specs.items():
            frame = self.frame(entry)
            state = states.get(group_id)
            if frame is None or state is None or getattr(state, "pose", None) is None:
                continue
            pose = poses.world_to_frame([*state.pose.position, *state.pose.orientation], frame)
            result[group_id] = SimpleNamespace(
                joints=state.joints, tcp=getattr(state, "tcp", None),
                pose=SimpleNamespace(position=pose[:3], orientation=pose[3:]),
            )  # fmt: skip
        return result

    def tcp_targets_to_world(self, chunk: ActionChunk) -> ActionChunk:
        """TCP targets from each action's coordinate system into the world."""
        specs = self._tcp_action_specs
        tcp = dict(chunk.tcp)
        changed = False
        for group_id, steps in chunk.tcp.items():
            entry = specs.get(group_id)
            frame = None if entry is None else self.frame(entry)
            if frame is not None:
                tcp[group_id] = [poses.frame_to_world(step, frame) for step in steps]
                changed = True
        return chunk.model_copy(update={"tcp": tcp}) if changed else chunk

    def apply_inverse_ops(self, chunk: ActionChunk) -> ActionChunk:
        """Convert an action chunk back into NOVA units; runs before relative targets (M4)."""
        joint_ops = self._joint_action_ops
        tcp_specs = self._tcp_action_specs
        if not joint_ops and not tcp_specs:
            return chunk

        joints = {
            group_id: [apply_ops_inverse(step, joint_ops[group_id]) for step in steps]
            if group_id in joint_ops
            else steps
            for group_id, steps in chunk.joints.items()
        }
        tcp = {
            group_id: [_inverse_tcp_step(step, tcp_specs[group_id]) for step in steps]
            if group_id in tcp_specs
            else steps
            for group_id, steps in chunk.tcp.items()
        }
        return chunk.model_copy(update={"joints": joints, "tcp": tcp})

    # -- Motion groups --

    def get_motion_groups(self) -> list[MotionGroup]:
        """All unique motion groups from observations and actions."""
        seen: set[str] = set()
        result: list[MotionGroup] = []
        for mg in self._iter_all_mgs():
            if mg.id not in seen:
                seen.add(mg.id)
                result.append(mg)
        return result

    def _iter_all_mgs(self) -> Iterator[MotionGroup]:
        for entry in [*self._observations, *self._actions]:
            yield from _groups(entry)

    # -- Schema introspection (used by executor, GR00T client) --

    @property
    def joint_mappings(self) -> list[_ObsJoints]:
        return [o for o in self._observations if isinstance(o, _ObsJoints)]

    @property
    def joint_action_keys(self) -> list[tuple[str, list[MotionGroup]]]:
        """Action-side joint keys: explicit Action.joint_positions() + inferred from action=True."""
        return [(e.key, _groups(e)) for e in self._action_entries(_ActJoints, _ObsJoints)]

    @property
    def tcp_mappings(self) -> list[_ObsTcp]:
        return [o for o in self._observations if isinstance(o, _ObsTcp)]

    @property
    def joint_velocity_mappings(self) -> list[_ObsJointVelocities]:
        return [o for o in self._observations if isinstance(o, _ObsJointVelocities)]

    @property
    def pose_mappings(self) -> list[_ObsPose]:
        return [o for o in self._observations if isinstance(o, _ObsPose)]

    @property
    def obs_io_mappings(self) -> list[_ObsIO]:
        return [o for o in self._observations if isinstance(o, _ObsIO)]

    @property
    def has_computed_actions(self) -> bool:
        """Whether any ``Action.computed`` hook is declared."""
        return any(isinstance(a, _ActComputed) for a in self._actions)

    @property
    def computed_action_keys(self) -> list[str]:
        """Action keys ``Action.computed`` hooks consume (``chunk.values``)."""
        return [k for a in self._actions if isinstance(a, _ActComputed) for k in a.keys]

    @property
    def computed_observations(self) -> list[_ObsComputed]:
        return [o for o in self._observations if isinstance(o, _ObsComputed)]

    @property
    def io_action_keys(self) -> list[tuple[str, MotionGroup, str, Mapping]]:
        """Action-side IO keys: explicit Action.io() + inferred from action=True."""
        return [
            (e.key, _groups(e)[0], e.io, e.mapping) for e in self._action_entries(_ActIO, _ObsIO)
        ]

    @property
    def tcp_action_keys(self) -> list[tuple[str, MotionGroup]]:
        """Action-side TCP keys: explicit Action.tcp() + inferred from action=True."""
        return [(e.key, _groups(e)[0]) for e in self._action_entries(_ActTcp, _ObsTcp)]

    @property
    def motion_action_keys(self) -> list[str]:
        """Every joint and TCP action key the policy is expected to answer with."""
        return [key for key, _ in self.joint_action_keys] + [key for key, _ in self.tcp_action_keys]

    @property
    def constants(self) -> dict[str, Any]:
        return {o.key: o.value for o in self._observations if isinstance(o, _ObsConstant)}

    @property
    def image_sources(self) -> dict[str, CameraSource]:
        return {o.key: o.source for o in self._observations if isinstance(o, _ObsImage)}

    @property
    def image_max_age_s(self) -> dict[str, float]:
        """Per-channel frame-age limits, for channels that declare one."""
        return {
            o.key: o.max_age_s
            for o in self._observations
            if isinstance(o, _ObsImage) and o.max_age_s is not None
        }

    @property
    def tcp(self) -> str:
        for o in self._observations:
            if isinstance(o, _ObsTcp) and o.tcp:
                return o.tcp
        return ""

    def relative_motion_groups(self) -> set[str]:
        """Motion group IDs where actions use relative mode."""
        return {
            mg.id
            for e in [*self._observations, *self._actions]
            if isinstance(e, (_ObsJoints, _ObsTcp, _ActJoints, _ActTcp)) and e.mode == "relative"
            for mg in _groups(e)
        }

    def relative_tcp_deltas(self) -> dict[str, TcpDelta]:
        """Motion group ID -> how its relative TCP action steps compose."""
        return {
            a.target.id: a.delta
            for a in self._actions
            if isinstance(a, _ActTcp) and a.mode == "relative"
        }

    def tcp_action_groups(self) -> dict[str, str]:
        """Motion group ID -> TCP name, for groups with a TCP action."""
        result: dict[str, str] = {}
        for a in self._actions:
            if isinstance(a, _ActTcp):
                result[a.target.id] = ""
        for o in self._observations:
            if isinstance(o, _ObsTcp) and o.action and o.source.id not in result:
                result[o.source.id] = o.tcp
        for o in self._observations:
            if isinstance(o, _ObsTcp) and o.source.id in result and o.tcp:
                result[o.source.id] = o.tcp
        return result

    def io_keys_by_controller(self) -> dict[str, list[str]]:
        """Hardware IO keys grouped by controller ID."""
        keys: dict[str, set[str]] = {}
        for o in self._observations:
            if isinstance(o, _ObsIO):
                keys.setdefault(get_controller_id(o.source), set()).add(o.io)
        for a in self._actions:
            if isinstance(a, _ActIO):
                keys.setdefault(get_controller_id(a.target), set()).add(a.io)
        return {k: sorted(v) for k, v in keys.items()}

    def observation_entry(self, key: str) -> ObservationEntry | None:
        """The observation entry named ``key``, if any."""
        for o in self._observations:
            if getattr(o, "key", None) == key:
                return o
        return None

    def observation_kind(self, key: str) -> str | None:
        """The kind of observation ``key`` (``"joint_positions"`` .. ``"computed"``), or None."""
        kinds: dict[type, str] = {
            _ObsJoints: "joint_positions",
            _ObsJointVelocities: "joint_velocities",
            _ObsTcp: "tcp",
            _ObsPose: "pose",
            _ObsIO: "io",
            _ObsImage: "image",
            _ObsConstant: "constant",
        }
        entry = self.observation_entry(key)
        if entry is not None:
            return kinds.get(type(entry))
        if any(key in o.keys for o in self.computed_observations):
            return "computed"
        return None

    def observation_vector(self, observation: dict[str, Any], key: str) -> list[float]:
        """The values of observation ``key`` from a built observation, as one vector:
        ``key_1 .. key_n`` for joints, the pose values for a TCP or pose, else ``key``."""
        entry = self.observation_entry(key)
        if isinstance(entry, (_ObsJoints, _ObsJointVelocities)):
            values: list[float] = []
            while f"{key}_{len(values) + 1}" in observation:
                values.append(float(observation[f"{key}_{len(values) + 1}"]))
            return values
        if isinstance(entry, (_ObsTcp, _ObsPose)):
            return [float(observation[name]) for name in tcp_value_names(entry)]
        value = observation[key]
        if isinstance(value, (int, float)):
            return [float(value)]
        return [float(v) for v in value]

    def observation_groups(self, observation: dict[str, Any]) -> dict[str, list[float]]:
        """A built observation as one vector per key; constants and text are left out.

        Entries whose robot state is missing are skipped; other numeric values (computed
        observations) keep their own names.
        """
        groups: dict[str, list[float]] = {}
        taken = {o.key for o in self._observations if isinstance(o, _ObsConstant)}
        for o in self._observations:
            if not isinstance(o, (_ObsJoints, _ObsJointVelocities, _ObsTcp, _ObsPose, _ObsIO)):
                continue
            if isinstance(o, (_ObsJoints, _ObsJointVelocities)):
                names = []
                while f"{o.key}_{len(names) + 1}" in observation:
                    names.append(f"{o.key}_{len(names) + 1}")
            else:
                names = tcp_value_names(o) if isinstance(o, (_ObsTcp, _ObsPose)) else [o.key]
            taken.update(names)
            if names and all(n in observation for n in names):
                groups[o.key] = [float(observation[n]) for n in names]
        for name, value in observation.items():
            if name in taken:
                continue
            values = value if isinstance(value, (list, tuple, np.ndarray)) else [value]
            try:
                groups[name] = [float(v) for v in np.ravel(values)]
            except (TypeError, ValueError):
                continue  # not numeric: no array to send
        return groups

    # -- Build observation --

    async def build_observation(
        self,
        states: dict[str, RobotState],
        io_values: dict[str, object] | None = None,
    ) -> dict[str, Any]:
        """Build the flat feature dict, kind by kind; computed values come last and see the rest."""
        obs: dict[str, Any] = {}
        for kind in _FILL_ORDER:
            for o in self._observations:
                if isinstance(o, kind):
                    await self._fill(obs, o, states, io_values)
        return obs

    async def _fill(  # ruff: ignore[complex-structure, too-many-branches]
        self,
        obs: dict[str, Any],
        o: ObservationEntry,
        states: dict[str, RobotState],
        io_values: dict[str, object] | None,
    ) -> None:
        """Add one observation entry's values to ``obs``."""
        if isinstance(o, (_ObsJoints, _ObsJointVelocities)):
            velocities = self._joint_velocities
            if isinstance(o, _ObsJointVelocities) and velocities is None:
                return
            values: list[float] = []
            for mg in o.sources:
                s = states.get(mg.id)
                if s is not None and isinstance(o, _ObsJoints):
                    values.extend(s.joints)
                elif s is not None and velocities is not None:
                    values.extend(velocities.read(mg.id, s.joints))
            for i, v in enumerate(apply_ops(values, o.ops), 1):
                obs[f"{o.key}_{i}"] = v
        elif isinstance(o, _ObsTcp):
            s = states.get(o.source.id)
            if s is not None and hasattr(s, "pose") and s.pose is not None:
                values = self.tcp_observation_values(o, s)
                obs.update(zip(tcp_value_names(o), values, strict=True))
        elif isinstance(o, _ObsPose):
            source = o.source
            if isinstance(source, Pose):
                pose = source
            elif isinstance(source, PoseSource):
                pose = source.read()
            else:
                pose = await source()
            values = _policy_pose_values([*pose.position, *pose.orientation], o)
            obs.update(zip(tcp_value_names(o), values, strict=True))
        elif isinstance(o, _ObsIO) and io_values is not None:
            raw = io_values.get(o.io)
            obs[o.key] = o.mapping.to_policy(raw) if isinstance(raw, (bool, int, float)) else 0.0
        elif isinstance(o, _ObsConstant):
            obs[o.key] = o.value
        elif isinstance(o, _ObsComputed):
            obs.update(await o.fn(obs))

    def action_columns(self, dof: dict[str, int]) -> list[tuple[str, int]]:
        """The flat action vector as ``(key, width)``: joints, TCP, IO, then computed keys."""
        columns = [(k, sum(dof.get(g.id, 0) for g in gs)) for k, gs in self.joint_action_keys]
        columns += [(k, self.tcp_action_width(mg.id)) for k, mg in self.tcp_action_keys]
        columns += [(k, 1) for k, *_ in self.io_action_keys]
        columns += [(k, 1) for k in self.computed_action_keys]
        return columns

    @staticmethod
    def split_actions(
        rows: list[list[float]], columns: list[tuple[str, int]], *, source: str = "The policy"
    ) -> dict[str, list[list[float]]]:
        """Split flat action rows into outputs by key; a row narrower than the columns raises."""
        width = sum(n for _, n in columns)
        outputs: dict[str, list[list[float]]] = {}
        offset = 0
        for key, n in columns:
            if n:
                outputs[key] = [list(row[offset : offset + n]) for row in rows]
            offset += n
        if rows and min(len(row) for row in rows) < width:
            msg = f"{source} answered {min(len(r) for r in rows)} values per step, expected {width}"
            raise ValueError(msg)
        return outputs

    @staticmethod
    def flatten(groups: dict[str, list[float]]) -> list[float]:
        """:meth:`observation_groups` as one vector, key after key, for a wire that takes one."""
        return [value for vector in groups.values() for value in vector]

    def unflatten(
        self, rows: list[list[float]], *, dof: dict[str, int], source: str = "The policy"
    ) -> dict[str, list[list[float]]]:
        """Flat action rows (one per step) as outputs by key, for :meth:`decode_actions`:
        :meth:`split_actions` over :meth:`action_columns`."""
        return self.split_actions(rows, self.action_columns(dof), source=source)

    def decode_actions(
        self,
        outputs: dict[str, Any],
        *,
        dt_ms: float,
        dof: dict[str, int] | None = None,
        action_timestep: int = -1,
        source: str = "The policy",
    ) -> ActionChunk:
        """A policy's outputs, by action key, as an :class:`ActionChunk` in the policy's units.

        Each output has shape ``(steps, width)`` (``(width,)`` for one step). A joint key shared
        by several groups splits into ``dof[group]`` columns each, in observation order (a last
        group of unknown dof takes the rest). An IO key takes its first step's value (M8);
        ``Action.computed`` keys land in ``values``; missing keys are skipped.

        Raises:
            ValueError: the answer is too narrow, or has none of the expected actions.
        """
        joints: dict[str, list[list[float]]] = {}
        for key, groups in self.joint_action_keys:
            if key not in outputs:
                continue
            steps = _steps(outputs[key])
            width = len(steps[0]) if steps else 0
            offset = 0
            for i, mg in enumerate(groups):
                n = (dof or {}).get(mg.id)
                if n is None and i == len(groups) - 1:
                    n = width - offset  # nothing measured to size it by: the rest
                if n == 0:
                    continue  # no joints observed for this group: it is not commanded
                if n is None or offset + n > width:
                    msg = (
                        f"{source}: action {key!r} has {width} values per step, which do not "
                        f"split into the joints of {[g.id for g in groups]} ({dict(dof or {})})"
                    )
                    raise ValueError(msg)
                joints[mg.id] = [step[offset : offset + n] for step in steps]
                offset += n
        tcp = {mg.id: _steps(outputs[key]) for key, mg in self.tcp_action_keys if key in outputs}
        ios: dict[str, dict[str, bool | int | float | str]] = {}
        for key, mg, io, mapping in self.io_action_keys:
            if key in outputs:
                first = np.asarray(outputs[key], dtype=np.float64).reshape(-1)[0]
                ios.setdefault(mg.id, {})[io] = mapping.to_hardware(float(first))
        values = {k: _steps(outputs[k]) for k in self.computed_action_keys if k in outputs}
        if self.motion_action_keys and not (joints or tcp or ios or values):
            msg = (
                f"{source} answered with none of the expected actions {self.motion_action_keys}; "
                f"got {sorted(outputs)}"
            )
            raise ValueError(msg)
        return ActionChunk(
            joints=joints,
            tcp=tcp,
            ios=ios or None,
            values=values,
            dt_ms=dt_ms,
            action_timestep=action_timestep,
        )

    async def run_computed_actions(self, action: ActionChunk) -> None:
        """Fire ``Action.computed`` side effects with the policy's chunk."""
        for a in self._actions:
            if isinstance(a, _ActComputed):
                await a.fn(action)


def _steps(output: object) -> list[list[float]]:
    """An output as a list of steps: ``(steps, width)``, or ``(width,)`` for one step."""
    array = np.asarray(output, dtype=np.float64)
    return (array.reshape(1, -1) if array.ndim <= 1 else array.reshape(len(array), -1)).tolist()


def _inverse_tcp_step(step: list[float], entry: _ObsTcp | _ActTcp) -> list[float]:
    """One policy TCP target back to NOVA's ``[x, y, z, rx, ry, rz]`` (mm, rotation vector)."""
    orientation_type = entry.orientation_type or poses.ROTATION_VECTOR
    if len(step) != 3 + poses.orientation_width(orientation_type):
        return step
    orientation = apply_ops_inverse(step[3:], entry.orientation_ops)
    return apply_ops_inverse(step[:3], entry.position_ops) + poses.to_rotvec(
        orientation, orientation_type
    )


def _policy_pose_values(pose: Sequence[float], entry: _ObsTcp | _ObsPose) -> list[float]:
    """A NOVA pose ``[x, y, z, rx, ry, rz]`` as the policy sees it."""
    orientation_type = entry.orientation_type or poses.ROTATION_VECTOR
    orientation = poses.from_rotvec(pose[3:6], orientation_type)
    return apply_ops(list(pose[:3]), entry.position_ops) + apply_ops(
        orientation, entry.orientation_ops
    )


def tcp_value_names(entry: _ObsTcp | _ActTcp | _ObsPose) -> list[str]:
    """Feature names: ``key_x .. key_rz``, or ``key_qx .. key_qw`` / ``key_e1 .. key_e3``."""
    orientation_type = entry.orientation_type or poses.ROTATION_VECTOR
    if orientation_type == poses.ROTATION_VECTOR:
        suffixes = _TCP_SUFFIXES
    elif orientation_type == poses.QUATERNION:
        suffixes = (*_TCP_SUFFIXES[:3], *_QUATERNION_SUFFIXES)
    else:
        suffixes = (*_TCP_SUFFIXES[:3], *_EULER_SUFFIXES)
    return [f"{entry.key}_{suffix}" for suffix in suffixes]


def tcp_width(entry: _ObsTcp | _ActTcp | _ObsPose) -> int:
    """Number of values of a TCP entry: 3 position + 3 or 4 orientation."""
    return 3 + poses.orientation_width(entry.orientation_type or poses.ROTATION_VECTOR)


def _check_orientation_ops(
    orientation_type: poses.OrientationType | None, ops: list[ValueOp] | None, where: str
) -> None:
    """Element-wise operators make no sense on quaternion components."""
    if ops and orientation_type == poses.QUATERNION:
        msg = f"{where}: orientation_ops cannot apply to quaternion components"
        raise ValueError(msg)
