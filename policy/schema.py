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
from typing import TYPE_CHECKING, Any

from policy._sdk import get_controller_id

_TCP_SUFFIXES = ("x", "y", "z", "rx", "ry", "rz")

if TYPE_CHECKING:
    from collections.abc import Iterator

    from nova.cell.motion_group import MotionGroup
    from nova.types import RobotState
    from policy.cameras import CameraSource
    from policy.types import ActionMode

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Value mappings
# ---------------------------------------------------------------------------


class Mapping:
    """Identity mapping — passes values through unchanged."""

    def to_policy(self, hardware_value: bool | int | float) -> float:  # noqa: FBT001
        if isinstance(hardware_value, bool):
            return 1.0 if hardware_value else 0.0
        return float(hardware_value)

    def to_hardware(self, policy_value: float) -> bool | int | float | str:
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

    def to_policy(self, hardware_value: bool | int | float) -> float:  # noqa: FBT001
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

    @property
    def sources(self) -> list[MotionGroup]:
        return self.source if isinstance(self.source, list) else [self.source]


@dataclass(slots=True)
class _ObsTcp:
    """TCP pose from a motion group. Position in mm, orientation as rotation vector (rad)."""

    key: str
    source: MotionGroup
    tcp: str = ""
    action: bool = False
    mode: ActionMode = "absolute"


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
    ) -> _ObsJoints:
        """Observe joint positions. Writable by default (infers matching action)."""
        return _ObsJoints(key=key, source=source, action=action, mode=mode)

    @staticmethod
    def tcp(
        key: str,
        source: MotionGroup,
        *,
        tcp: str = "",
        action: bool = False,
        mode: ActionMode = "absolute",
    ) -> _ObsTcp:
        """Observe TCP pose [x, y, z, rx, ry, rz] in mm / rad (Nova native).

        Set action=True to control via TCP waypoint jogging.
        """
        return _ObsTcp(key=key, source=source, tcp=tcp, action=action, mode=mode)

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
    def image(key: str, source: CameraSource) -> _ObsImage:
        """Observe a camera image. Source must have connect/read/disconnect."""
        return _ObsImage(key=key, source=source)

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

    @property
    def targets(self) -> list[MotionGroup]:
        return self.target if isinstance(self.target, list) else [self.target]


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


ComputedActFn = Callable[[dict[str, Any]], Awaitable[None]]


@dataclass(slots=True)
class _ActComputed:
    """Async function called when policy returns: ``async (action_dict) -> None``."""

    fn: ComputedActFn


ActionEntry = _ActJoints | _ActTcp | _ActIO | _ActComputed


class Action:
    """Factory for explicit action entries."""

    @staticmethod
    def joint_positions(
        key: str, target: MotionGroup | list[MotionGroup], *, mode: ActionMode = "absolute"
    ) -> _ActJoints:
        """Joint action with a key different from the observation."""
        return _ActJoints(key=key, target=target, mode=mode)

    @staticmethod
    def tcp(key: str, target: MotionGroup, *, mode: ActionMode = "absolute") -> _ActTcp:
        """TCP pose action — executor uses Cartesian waypoint jogging."""
        return _ActTcp(key=key, target=target, mode=mode)

    @staticmethod
    def io(key: str, target: MotionGroup, io: str, *, mapping: Mapping | None = None) -> _ActIO:
        """IO write action."""
        return _ActIO(key=key, target=target, io=io, mapping=mapping or Mapping())

    @staticmethod
    def computed(fn: ComputedActFn) -> _ActComputed:
        """Async function for side effects when the policy returns.

        Example::

            async def update_conveyor(action: dict) -> None:
                speed = action.get("conveyor_speed", 0.0)
                await plc.write_register(100, speed)

            schema = PolicySchema(
                observations=[Observation.joint_positions("arm", source=mg)],
                actions=[Action.computed(update_conveyor)],
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
        seen_act: set[str] = set()
        for a in self._actions:
            k = getattr(a, "key", None)
            if k is not None:
                if k in seen_act:
                    msg = f"Duplicate action key: {k!r}"
                    raise ValueError(msg)
                seen_act.add(k)

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
            for i, v in enumerate(joints, 1):
                obs[f"{o.key}_{i}"] = v

    def _fill_tcp(self, obs: dict[str, Any], states: dict[str, RobotState]) -> None:
        for o in self._observations:
            if not isinstance(o, _ObsTcp):
                continue
            s = states.get(o.source.id)
            if s is not None and hasattr(s, "pose") and s.pose is not None:
                values = list(s.pose.position) + list(s.pose.orientation)
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

    # -- Parse action --

    async def parse_action(
        self,
        action: dict[str, float],
    ) -> tuple[
        dict[str, list[list[float]]],
        dict[str, list[list[float]]],
        dict[str, dict[str, bool | int | float | str]] | None,
    ]:
        """Parse a flat action dict into per-MG joints, TCP targets, and IOs."""
        explicit_keys = {a.key for a in self._actions if hasattr(a, "key")}
        joints = self._parse_joints(action, explicit_keys)
        tcp_targets = self._parse_tcp(action, explicit_keys)
        ios = self._parse_ios(action, explicit_keys)

        # Computed side effects
        for a in self._actions:
            if isinstance(a, _ActComputed):
                await a.fn(action)

        return joints, tcp_targets, ios or None

    def _parse_joints(
        self,
        action: dict[str, float],
        explicit_keys: set[str],
    ) -> dict[str, list[list[float]]]:
        joints: dict[str, list[list[float]]] = {}
        for key, sources, _mode in self._joint_action_sources(explicit_keys):
            values = _extract_indexed(action, key)
            if not values:
                continue
            if len(sources) == 1:
                joints[sources[0].id] = [values]
            else:
                per_mg = len(values) // len(sources)
                for i, mg in enumerate(sources):
                    chunk = values[i * per_mg : (i + 1) * per_mg]
                    if chunk:
                        joints[mg.id] = [chunk]
        return joints

    def _parse_ios(
        self,
        action: dict[str, float],
        explicit_keys: set[str],
    ) -> dict[str, dict[str, bool | int | float | str]]:
        ios: dict[str, dict[str, bool | int | float | str]] = {}
        for key, mg, hw_key, mapping in self._io_action_sources(explicit_keys):
            if key in action:
                ios.setdefault(mg.id, {})[hw_key] = mapping.to_hardware(float(action[key]))
        return ios

    def _parse_tcp(
        self,
        action: dict[str, float],
        explicit_keys: set[str],
    ) -> dict[str, list[list[float]]]:
        tcp_targets: dict[str, list[list[float]]] = {}
        for key, mg in self._tcp_action_sources(explicit_keys):
            values = _extract_tcp(action, key)
            if values:
                tcp_targets[mg.id] = [values]
        return tcp_targets

    def _joint_action_sources(
        self, explicit_keys: set[str]
    ) -> Iterator[tuple[str, list[MotionGroup], ActionMode]]:
        for a in self._actions:
            if isinstance(a, _ActJoints):
                yield a.key, a.targets, a.mode
        for o in self._observations:
            if isinstance(o, _ObsJoints) and o.action and o.key not in explicit_keys:
                yield o.key, o.sources, o.mode

    def _io_action_sources(
        self, explicit_keys: set[str]
    ) -> Iterator[tuple[str, MotionGroup, str, Mapping]]:
        for a in self._actions:
            if isinstance(a, _ActIO):
                yield a.key, a.target, a.io, a.mapping
        for o in self._observations:
            if isinstance(o, _ObsIO) and o.action and o.key not in explicit_keys:
                yield o.key, o.source, o.io, o.mapping

    def _tcp_action_sources(self, explicit_keys: set[str]) -> Iterator[tuple[str, MotionGroup]]:
        for a in self._actions:
            if isinstance(a, _ActTcp):
                yield a.key, a.target
        for o in self._observations:
            if isinstance(o, _ObsTcp) and o.action and o.key not in explicit_keys:
                yield o.key, o.source


def _extract_indexed(d: dict[str, float], prefix: str) -> list[float]:
    """Extract {prefix}_1, {prefix}_2, ... from a dict."""
    values: list[float] = []
    i = 1
    while f"{prefix}_{i}" in d:
        values.append(float(d[f"{prefix}_{i}"]))
        i += 1
    return values


def _extract_tcp(d: dict[str, float], prefix: str) -> list[float]:
    """Extract {prefix}_x, {prefix}_y, ..., {prefix}_rz from a dict."""
    if f"{prefix}_x" not in d:
        return []
    return [float(d[f"{prefix}_{s}"]) for s in _TCP_SUFFIXES]
