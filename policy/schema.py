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

import logging
from collections.abc import Awaitable, Callable
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any

from policy._sdk import get_controller_id
from policy.pose import TcpFormat, pose_to_tcp

if TYPE_CHECKING:
    from nova.cell.motion_group import MotionGroup
    from nova.types import RobotState

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Value mappings
# ---------------------------------------------------------------------------

class Mapping:
    """Identity mapping — passes values through unchanged."""

    def to_policy(self, hardware_value: object) -> float:
        if isinstance(hardware_value, bool):
            return 1.0 if hardware_value else 0.0
        return float(hardware_value)  # type: ignore[arg-type]

    def to_hardware(self, policy_value: float) -> bool | int | float | str:
        return policy_value

IdentityMapping = Mapping


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

    def to_policy(self, hardware_value: object) -> float:
        if isinstance(hardware_value, bool):
            return self.on if hardware_value else self.off
        return self.on if float(hardware_value) >= self.threshold else self.off  # type: ignore[arg-type]

    def to_hardware(self, policy_value: float) -> bool:
        return policy_value >= self.threshold


# ---------------------------------------------------------------------------
# Observation entries (created via Observation factory)
# ---------------------------------------------------------------------------

@dataclass
class _ObsJoints:
    """Joint positions from one or more motion groups."""
    key: str
    source: MotionGroup | list[MotionGroup]
    writable: bool = True
    mode: str = "absolute"

    @property
    def sources(self) -> list[MotionGroup]:
        return self.source if isinstance(self.source, list) else [self.source]


@dataclass
class _ObsJointSignal:
    """Optional per-joint signal (torques/currents). Read-only."""
    key: str
    source: MotionGroup
    field_name: str
    default: list[float] | None = None


@dataclass
class _ObsTcp:
    """TCP pose from a motion group."""
    key: str
    source: MotionGroup
    tcp: str = ""
    format: TcpFormat = TcpFormat.ROTATION_VECTOR


@dataclass
class _ObsIO:
    """IO value (digital/analog). Writable by default."""
    key: str
    source: MotionGroup
    io: str
    mapping: Mapping = field(default_factory=Mapping)
    writable: bool = True


@dataclass
class _ObsImage:
    """Camera image from a CameraSource."""
    key: str
    source: object


@dataclass
class _ObsConstant:
    """Fixed value in every observation."""
    key: str
    value: Any


ComputedObsFn = Callable[[dict[str, Any]], Awaitable[dict[str, Any]]]


@dataclass
class _ObsComputed:
    """Async function called each step: ``async (obs_so_far) -> dict``."""
    fn: ComputedObsFn


ObservationEntry = (
    _ObsJoints | _ObsJointSignal | _ObsTcp | _ObsIO
    | _ObsImage | _ObsConstant | _ObsComputed
)


class Observation:
    """Factory for observation entries."""

    @staticmethod
    def joint_positions(
        key: str, source: MotionGroup | list[MotionGroup], *,
        writable: bool = True, mode: str = "absolute",
    ) -> _ObsJoints:
        """Observe joint positions. Writable by default (infers matching action)."""
        return _ObsJoints(key=key, source=source, writable=writable, mode=mode)

    @staticmethod
    def joint_torques(key: str, source: MotionGroup, *, default: list[float] | None = None) -> _ObsJointSignal:
        """Observe joint torques (read-only)."""
        return _ObsJointSignal(key=key, source=source, field_name="joint_torques", default=default)

    @staticmethod
    def joint_currents(key: str, source: MotionGroup, *, default: list[float] | None = None) -> _ObsJointSignal:
        """Observe joint currents (read-only)."""
        return _ObsJointSignal(key=key, source=source, field_name="joint_currents", default=default)

    @staticmethod
    def tcp(
        key: str, source: MotionGroup, *, tcp: str = "",
        format: TcpFormat = TcpFormat.ROTATION_VECTOR,
    ) -> _ObsTcp:
        """Observe TCP pose in the given format."""
        return _ObsTcp(key=key, source=source, tcp=tcp, format=format)

    @staticmethod
    def io(
        key: str, source: MotionGroup, io: str, *,
        mapping: Mapping | None = None, writable: bool = True,
    ) -> _ObsIO:
        """Observe an IO value. Writable by default (policy can write it back)."""
        return _ObsIO(key=key, source=source, io=io, mapping=mapping or Mapping(), writable=writable)

    @staticmethod
    def image(key: str, source: object) -> _ObsImage:
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

@dataclass
class _ActJoints:
    """Explicit joint position action."""
    key: str
    target: MotionGroup | list[MotionGroup]
    mode: str = "absolute"

    @property
    def targets(self) -> list[MotionGroup]:
        return self.target if isinstance(self.target, list) else [self.target]


@dataclass
class _ActIO:
    """Explicit IO write action."""
    key: str
    target: MotionGroup
    io: str
    mapping: Mapping = field(default_factory=Mapping)


ComputedActFn = Callable[[dict[str, Any]], Awaitable[None]]


@dataclass
class _ActComputed:
    """Async function called when policy returns: ``async (action_dict) -> None``."""
    fn: ComputedActFn


ActionEntry = _ActJoints | _ActIO | _ActComputed


class Action:
    """Factory for explicit action entries."""

    @staticmethod
    def joint_positions(key: str, target: MotionGroup | list[MotionGroup], *, mode: str = "absolute") -> _ActJoints:
        """Joint action with a key different from the observation."""
        return _ActJoints(key=key, target=target, mode=mode)

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
            k = o.key if hasattr(o, "key") else None
            if k is not None:
                if k in seen:
                    msg = f"Duplicate observation key: {k!r}"
                    raise ValueError(msg)
                seen.add(k)
        seen_act: set[str] = set()
        for a in self._actions:
            k = a.key if hasattr(a, "key") else None
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

    def _iter_all_mgs(self):  # noqa: ANN202
        for o in self._observations:
            if isinstance(o, _ObsJoints):
                yield from o.sources
            elif isinstance(o, (_ObsTcp, _ObsIO, _ObsJointSignal)):
                yield o.source
        for a in self._actions:
            if isinstance(a, _ActJoints):
                yield from a.targets
            elif isinstance(a, _ActIO):
                yield a.target

    # -- Schema introspection (used by executor, GR00T client) --

    @property
    def joint_mappings(self) -> list[_ObsJoints]:
        return [o for o in self._observations if isinstance(o, _ObsJoints)]

    @property
    def tcp_mappings(self) -> list[_ObsTcp]:
        return [o for o in self._observations if isinstance(o, _ObsTcp)]

    @property
    def obs_io_mappings(self) -> list[_ObsIO]:
        return [o for o in self._observations if isinstance(o, _ObsIO)]

    @property
    def constants(self) -> dict[str, Any]:
        return {o.key: o.value for o in self._observations if isinstance(o, _ObsConstant)}

    @property
    def image_sources(self) -> dict[str, object]:
        return {o.key: o.source for o in self._observations if isinstance(o, _ObsImage)}

    @property
    def tcp(self) -> str:
        for o in self._observations:
            if isinstance(o, _ObsTcp) and o.tcp:
                return o.tcp
        return ""

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
        self, states: dict[str, RobotState], io_values: dict[str, object] | None = None,
    ) -> dict[str, Any]:
        """Build a flat feature dict from robot states + IO values."""
        obs: dict[str, Any] = {}
        self._fill_joints(obs, states)
        self._fill_tcp(obs, states)
        self._fill_joint_signals(obs, states)
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
                for i, v in enumerate(pose_to_tcp(s.pose, o.format), 1):
                    obs[f"{o.key}_{i}"] = v

    def _fill_joint_signals(self, obs: dict[str, Any], states: dict[str, RobotState]) -> None:
        for o in self._observations:
            if not isinstance(o, _ObsJointSignal):
                continue
            s = states.get(o.source.id)
            values = getattr(s, o.field_name, None) if s is not None else None
            data = values or (tuple(o.default) if o.default else None)
            if data is not None:
                for i, v in enumerate(data, 1):
                    obs[f"{o.key}_{i}"] = v

    def _fill_ios(self, obs: dict[str, Any], io_values: dict[str, object] | None) -> None:
        if io_values is None:
            return
        for o in self._observations:
            if isinstance(o, _ObsIO):
                raw = io_values.get(o.io)
                obs[o.key] = o.mapping.to_policy(raw) if raw is not None else 0.0

    def _fill_constants(self, obs: dict[str, Any]) -> None:
        for o in self._observations:
            if isinstance(o, _ObsConstant):
                obs[o.key] = o.value

    async def _fill_computed(self, obs: dict[str, Any]) -> None:
        for o in self._observations:
            if isinstance(o, _ObsComputed):
                extra = await o.fn(obs)
                if isinstance(extra, dict):
                    obs.update(extra)

    # -- Parse action --

    async def parse_action(self, action: dict[str, float]) -> tuple[
        dict[str, list[list[float]]],
        dict[str, dict[str, bool | int | float | str]] | None,
    ]:
        """Parse a flat action dict into per-MG joints and IOs."""
        explicit_keys = {a.key for a in self._actions if hasattr(a, "key")}
        joints: dict[str, list[list[float]]] = {}
        ios: dict[str, dict[str, bool | int | float | str]] = {}

        # Joint actions: explicit + writable observations
        for key, sources in self._joint_action_sources(explicit_keys):
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

        # IO actions: explicit + writable observations
        for key, mg, hw_key, mapping in self._io_action_sources(explicit_keys):
            if key in action:
                ios.setdefault(mg.id, {})[hw_key] = mapping.to_hardware(float(action[key]))

        # Computed side effects
        for a in self._actions:
            if isinstance(a, _ActComputed):
                await a.fn(action)

        return joints, ios or None

    def _joint_action_sources(self, explicit_keys: set[str]):  # noqa: ANN202
        for a in self._actions:
            if isinstance(a, _ActJoints):
                yield a.key, a.targets
        for o in self._observations:
            if isinstance(o, _ObsJoints) and o.writable and o.key not in explicit_keys:
                yield o.key, o.sources

    def _io_action_sources(self, explicit_keys: set[str]):  # noqa: ANN202
        for a in self._actions:
            if isinstance(a, _ActIO):
                yield a.key, a.target, a.io, a.mapping
        for o in self._observations:
            if isinstance(o, _ObsIO) and o.writable and o.key not in explicit_keys:
                yield o.key, o.source, o.io, o.mapping


def _extract_indexed(d: dict[str, float], prefix: str) -> list[float]:
    """Extract {prefix}_1, {prefix}_2, ... from a dict."""
    values: list[float] = []
    i = 1
    while f"{prefix}_{i}" in d:
        values.append(float(d[f"{prefix}_{i}"]))
        i += 1
    return values
