"""PolicySchema — declares what the policy observes and controls.

Example::

    schema = PolicySchema(observations=[
        Observation.joint_positions("left_joints", source=mg_left),
        Observation.joint_positions("right_joints", source=mg_right),
        Observation.io("gripper", source=mg_left, io="digital_out[0]",
                       mapping=BoolMapping(on=100.0)),
        Observation.image("cam", source=cameras.device("12345")),
        Observation.constant("language", value="Pick up the box."),
        Observation.computed(read_force_sensor),
    ])

Writable observations (default) automatically infer matching actions.
Use explicit ``Action`` entries only when the action key differs.
"""

from __future__ import annotations

from collections.abc import Awaitable, Callable
from dataclasses import dataclass, field
import logging
from typing import TYPE_CHECKING, Any, cast

from novapolicy._sdk import get_controller_id
from novapolicy.ops import apply_ops, apply_ops_inverse, reject_forward_only

_TCP_SUFFIXES = ("x", "y", "z", "rx", "ry", "rz")

if TYPE_CHECKING:
    from collections.abc import Iterator

    from nova.cell.motion_group import MotionGroup
    from nova.types import RobotState
    from novapolicy.cameras import CameraSource
    from novapolicy.ops import ValueOp
    from novapolicy.types import ActionChunk, ActionMode

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Value mappings
# ---------------------------------------------------------------------------


class Mapping:
    """Identity mapping — passes values through unchanged.

    Distinct from :class:`~novapolicy.ops.ValueOp` on purpose. A ``Mapping``
    crosses a *type* boundary — hardware bools and analogue levels on one side,
    the policy's floats on the other — and is applied by each policy client as
    it decodes its own wire format. A ``ValueOp`` is a float-to-float *units*
    transform applied by the schema itself, in both directions, at one seam.
    Merging them would make either job worse.
    """

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
class _ObsTcp:
    """TCP pose from a motion group. Position in mm, orientation as rotation vector (rad)."""

    key: str
    source: MotionGroup
    tcp: str = ""
    action: bool = False
    mode: ActionMode = "absolute"
    position_ops: list[ValueOp] = field(default_factory=list)
    orientation_ops: list[ValueOp] = field(default_factory=list)


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


ObservationEntry = _ObsJoints | _ObsTcp | _ObsIO | _ObsImage | _ObsConstant | _ObsComputed


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
            ops: Unit conversions applied to every joint, front-to-back on the
                observation and inverted back-to-front on the action. Use when
                the training dataset was not recorded in NOVA's radians, e.g.
                ``ops=[Rad2Deg()]``.
        """
        return _ObsJoints(key=key, source=source, action=action, mode=mode, ops=list(ops or []))

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
    ) -> _ObsTcp:
        """Observe TCP pose [x, y, z, rx, ry, rz] in mm / rad (Nova native).

        Set action=True to control via TCP waypoint jogging.

        Position and orientation carry different units, so they take separate
        operator lists: ``position_ops=[Scale(0.001)]`` converts millimetres to
        metres without touching the rotation vector.
        """
        return _ObsTcp(
            key=key,
            source=source,
            tcp=tcp,
            action=action,
            mode=mode,
            position_ops=list(position_ops or []),
            orientation_ops=list(orientation_ops or []),
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
            max_age_s: Oldest frame this channel may contribute, in seconds.
                Defaults to the executor's ``camera_max_age_s``. Set it per
                channel when one camera runs at a different rate than the rest.
        """
        return _ObsImage(key=key, source=source, max_age_s=max_age_s)

    @staticmethod
    def constant(key: str, value: object) -> _ObsConstant:
        """Fixed value in every observation (e.g. language instruction)."""
        return _ObsConstant(key=key, value=value)

    @staticmethod
    def computed(fn: ComputedObsFn) -> _ObsComputed:
        """Async function called each step to add external data to the observation.

        Example::

            async def read_force_sensor(obs: dict) -> dict:
                force = await sensor.read()
                return {"force_x": force[0], "force_y": force[1], "force_z": force[2]}

            schema = PolicySchema(observations=[
                Observation.joint_positions("arm", source=mg),
                Observation.computed(read_force_sensor),
            ])
        """
        return _ObsComputed(fn=fn)


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


ComputedActFn = Callable[["ActionChunk"], Awaitable[None]]


@dataclass(slots=True)
class _ActComputed:
    """Async side-effect called with the policy's chunk: ``async (chunk) -> None``."""

    fn: ComputedActFn


ActionEntry = _ActJoints | _ActTcp | _ActIO | _ActComputed


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
        """Joint action with a key different from the observation.

        ``ops`` are inverted on the way to the robot. An explicit action's
        operators win over those of an observation sharing its key.
        """
        return _ActJoints(key=key, target=target, mode=mode, ops=list(ops or []))

    @staticmethod
    def tcp(
        key: str,
        target: MotionGroup,
        *,
        mode: ActionMode = "absolute",
        position_ops: list[ValueOp] | None = None,
        orientation_ops: list[ValueOp] | None = None,
    ) -> _ActTcp:
        """TCP pose action — executor uses Cartesian waypoint jogging."""
        return _ActTcp(
            key=key,
            target=target,
            mode=mode,
            position_ops=list(position_ops or []),
            orientation_ops=list(orientation_ops or []),
        )

    @staticmethod
    def io(key: str, target: MotionGroup, io: str, *, mapping: Mapping | None = None) -> _ActIO:
        """IO write action."""
        return _ActIO(key=key, target=target, io=io, mapping=mapping or Mapping())

    @staticmethod
    def computed(fn: ComputedActFn) -> _ActComputed:
        """Async side effect run after each policy call, with the returned chunk.

        Example::

            async def journal(chunk: ActionChunk) -> None:
                await db.write(chunk.joints)

            schema = PolicySchema(
                observations=[Observation.joint_positions("arm", source=mg)],
                actions=[Action.computed(journal)],
            )
        """
        return _ActComputed(fn=fn)


# ---------------------------------------------------------------------------
# PolicySchema
# ---------------------------------------------------------------------------


class PolicySchema:
    """Declares what a policy observes and controls."""

    def __init__(
        self,
        observations: list[ObservationEntry],
        actions: list[ActionEntry] | None = None,
    ) -> None:
        self._observations = list(observations)
        self._actions = list(actions or [])
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
        """Refuse operators that cannot be inverted on a channel the policy writes.

        Direction is a property of the operator class, so nothing is probed:
        library-owned operators are invertible by construction, and any that
        declare themselves forward-only have no reverse for the executor to
        send back to the robot.
        """
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
        """Motion group id → joint operators to invert on the action path.

        Resolution follows the same explicit-wins rule as ``joint_action_keys``:
        an explicit ``Action.joint_positions`` supplies its own operators, and
        otherwise they come from the observation sharing its key.
        """
        by_key: dict[str, list[ValueOp]] = {
            o.key: o.ops for o in self._observations if isinstance(o, _ObsJoints)
        }
        by_key.update({a.key: a.ops for a in self._actions if isinstance(a, _ActJoints)})

        result: dict[str, list[ValueOp]] = {}
        for key, motion_groups in self.joint_action_keys:
            ops = by_key.get(key)
            if ops:
                for mg in motion_groups:
                    result[mg.id] = ops
        return result

    @property
    def _tcp_action_ops(self) -> dict[str, tuple[list[ValueOp], list[ValueOp]]]:
        """Motion group id → ``(position_ops, orientation_ops)`` for the action path."""
        by_key: dict[str, tuple[list[ValueOp], list[ValueOp]]] = {
            o.key: (o.position_ops, o.orientation_ops)
            for o in self._observations
            if isinstance(o, _ObsTcp)
        }
        by_key.update({
            a.key: (a.position_ops, a.orientation_ops)
            for a in self._actions
            if isinstance(a, _ActTcp)
        })

        result: dict[str, tuple[list[ValueOp], list[ValueOp]]] = {}
        for key, motion_group in self.tcp_action_keys:
            ops = by_key.get(key)
            if ops and (ops[0] or ops[1]):
                result[motion_group.id] = ops
        return result

    def apply_inverse_ops(self, chunk: ActionChunk) -> ActionChunk:
        """Convert a policy's action chunk back into NOVA's own units.

        Must run **before** relative targets are resolved: a delta expressed in
        degrees added to a state in radians is exactly the error these operators
        exist to prevent.
        """
        joint_ops = self._joint_action_ops
        tcp_ops = self._tcp_action_ops
        if not joint_ops and not tcp_ops:
            return chunk

        joints = {
            group_id: [apply_ops_inverse(step, joint_ops[group_id]) for step in steps]
            if group_id in joint_ops
            else steps
            for group_id, steps in chunk.joints.items()
        }
        tcp = {
            group_id: [_inverse_tcp_step(step, *tcp_ops[group_id]) for step in steps]
            if group_id in tcp_ops
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
        for o in self._observations:
            if isinstance(o, _ObsJoints):
                yield from o.sources
            elif isinstance(o, (_ObsTcp, _ObsIO)):
                yield o.source
        for a in self._actions:
            if isinstance(a, _ActJoints):
                yield from a.targets
            elif isinstance(a, (_ActTcp, _ActIO)):
                yield a.target

    # -- Schema introspection (used by executor, GR00T client) --

    @property
    def joint_mappings(self) -> list[_ObsJoints]:
        return [o for o in self._observations if isinstance(o, _ObsJoints)]

    @property
    def joint_action_keys(self) -> list[tuple[str, list[MotionGroup]]]:
        """Action-side joint keys: explicit Action.joint_positions() + inferred from action=True."""
        explicit_keys = {a.key for a in self._actions if isinstance(a, _ActJoints)}
        result: list[tuple[str, list[MotionGroup]]] = [
            (a.key, a.targets) for a in self._actions if isinstance(a, _ActJoints)
        ]
        result.extend(
            (o.key, o.sources)
            for o in self._observations
            if isinstance(o, _ObsJoints) and o.action and o.key not in explicit_keys
        )
        return result

    @property
    def tcp_mappings(self) -> list[_ObsTcp]:
        return [o for o in self._observations if isinstance(o, _ObsTcp)]

    @property
    def obs_io_mappings(self) -> list[_ObsIO]:
        return [o for o in self._observations if isinstance(o, _ObsIO)]

    @property
    def computed_observations(self) -> list[_ObsComputed]:
        return [o for o in self._observations if isinstance(o, _ObsComputed)]

    @property
    def io_action_keys(self) -> list[tuple[str, MotionGroup, str, Mapping]]:
        """Action-side IO keys: explicit Action.io() + inferred from action=True."""
        explicit_keys = {a.key for a in self._actions if isinstance(a, _ActIO)}
        result: list[tuple[str, MotionGroup, str, Mapping]] = [
            (a.key, a.target, a.io, a.mapping) for a in self._actions if isinstance(a, _ActIO)
        ]
        result.extend(
            (o.key, o.source, o.io, o.mapping)
            for o in self._observations
            if isinstance(o, _ObsIO) and o.action and o.key not in explicit_keys
        )
        return result

    @property
    def tcp_action_keys(self) -> list[tuple[str, MotionGroup]]:
        """Action-side TCP keys: explicit Action.tcp() + inferred from action=True."""
        explicit_keys = {a.key for a in self._actions if isinstance(a, _ActTcp)}
        result: list[tuple[str, MotionGroup]] = [
            (a.key, a.target) for a in self._actions if isinstance(a, _ActTcp)
        ]
        result.extend(
            (o.key, o.source)
            for o in self._observations
            if isinstance(o, _ObsTcp) and o.action and o.key not in explicit_keys
        )
        return result

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
        result: set[str] = set()
        for o in self._observations:
            if isinstance(o, (_ObsJoints, _ObsTcp)) and o.mode == "relative":
                if isinstance(o, _ObsJoints):
                    result.update(mg.id for mg in o.sources)
                else:
                    result.add(o.source.id)
        for a in self._actions:
            if isinstance(a, (_ActJoints, _ActTcp)) and a.mode == "relative":
                if isinstance(a, _ActJoints):
                    result.update(mg.id for mg in a.targets)
                else:
                    result.add(a.target.id)
        return result

    def tcp_action_groups(self) -> dict[str, str]:
        """Motion group IDs that use TCP waypoint jogging → tcp name.

        Returns dict mapping motion group ID to TCP name.
        """
        result: dict[str, str] = {}
        # Explicit Action.tcp()
        for a in self._actions:
            if isinstance(a, _ActTcp):
                result[a.target.id] = ""
        # Writable Observation.tcp()
        for o in self._observations:
            if isinstance(o, _ObsTcp) and o.action and o.source.id not in result:
                result[o.source.id] = o.tcp
        # Fill in tcp names from observations
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

    # -- Build observation --

    async def build_observation(
        self,
        states: dict[str, RobotState],
        io_values: dict[str, object] | None = None,
    ) -> dict[str, Any]:
        """Build a flat feature dict from robot states + IO values."""
        obs: dict[str, Any] = {}
        self._fill_joints(obs, states)
        self._fill_tcp(obs, states)
        self._fill_ios(obs, io_values)
        self._fill_constants(obs)
        await self._fill_computed(obs)
        return obs

    def _fill_joints(self, obs: dict[str, Any], states: dict[str, RobotState]) -> None:
        for o in self._observations:
            if not isinstance(o, _ObsJoints):
                continue
            joints: list[float] = []
            for mg in o.sources:
                s = states.get(mg.id)
                if s is not None:
                    joints.extend(s.joints)
            for i, v in enumerate(apply_ops(joints, o.ops), 1):
                obs[f"{o.key}_{i}"] = v

    def _fill_tcp(self, obs: dict[str, Any], states: dict[str, RobotState]) -> None:
        for o in self._observations:
            if not isinstance(o, _ObsTcp):
                continue
            s = states.get(o.source.id)
            if s is not None and hasattr(s, "pose") and s.pose is not None:
                values = apply_ops(list(s.pose.position), o.position_ops) + apply_ops(
                    list(s.pose.orientation), o.orientation_ops
                )
                for suffix, v in zip(_TCP_SUFFIXES, values, strict=True):
                    obs[f"{o.key}_{suffix}"] = v

    def _fill_ios(self, obs: dict[str, Any], io_values: dict[str, object] | None) -> None:
        if io_values is None:
            return
        for o in self._observations:
            if isinstance(o, _ObsIO):
                raw = io_values.get(o.io)
                if raw is not None and isinstance(raw, (bool, int, float)):
                    obs[o.key] = o.mapping.to_policy(raw)
                else:
                    obs[o.key] = 0.0

    def _fill_constants(self, obs: dict[str, Any]) -> None:
        for o in self._observations:
            if isinstance(o, _ObsConstant):
                obs[o.key] = o.value

    async def _fill_computed(self, obs: dict[str, Any]) -> None:
        for o in self._observations:
            if isinstance(o, _ObsComputed):
                extra = await o.fn(obs)
                obs.update(extra)

    async def run_computed_actions(self, action: ActionChunk) -> None:
        """Fire ``Action.computed`` side effects with the policy's chunk."""
        for a in self._actions:
            if isinstance(a, _ActComputed):
                await a.fn(action)


def _inverse_tcp_step(
    step: list[float],
    position_ops: list[ValueOp],
    orientation_ops: list[ValueOp],
) -> list[float]:
    """Invert one ``[x, y, z, rx, ry, rz]`` target, position and rotation apart."""
    if len(step) != len(_TCP_SUFFIXES):
        return step
    return apply_ops_inverse(step[:3], position_ops) + apply_ops_inverse(step[3:], orientation_ops)
