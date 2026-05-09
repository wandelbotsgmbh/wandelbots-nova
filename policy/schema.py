"""PolicySchema — user-facing API for declaring policy observations and actions.

Users describe what their policy observes and what it controls:

    schema = PolicySchema(
        observations=[
            Observation.joint_positions("left_joints", source=mg_left),
            Observation.joint_positions("right_joints", source=mg_right),
            Observation.tcp("left_eef_9d", source=mg_left, tcp="Flange", format=TcpFormat.ROT6D),
            Observation.io("gripper", source=mg_left, io="digital_out[0]",
                           mapping=BoolMapping(true_value=100.0)),
            Observation.image("context_camera", source=context_camera_source),
            Observation.constant("language", value="Pick up the box."),
        ],
    )

Joint actions and IO actions are inferred from writable observations (the
default). Set ``writable=False`` for read-only entries. Use explicit
``Action`` entries only when the action key or hardware target differs from
the observation.
"""

from __future__ import annotations

import logging
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
    """Base class for value mappings between hardware and policy representation."""

    def to_policy(self, hardware_value: object) -> float:
        """Convert a hardware value to its policy representation."""
        if isinstance(hardware_value, bool):
            return 1.0 if hardware_value else 0.0
        return float(hardware_value)  # type: ignore[arg-type]

    def to_hardware(self, policy_value: float) -> bool | int | float | str:
        """Convert a policy value to its hardware representation."""
        return policy_value


class IdentityMapping(Mapping):
    """Pass-through mapping. Policy and hardware values are the same."""


class BoolMapping(Mapping):
    """Map between hardware bool and policy float.

    Parameters
    ----------
    on:
        Policy value when hardware is ``True``.
    off:
        Policy value when hardware is ``False``.
    threshold:
        Policy values >= threshold write hardware ``True``.
        Defaults to the midpoint between ``off`` and ``on``.
    """

    def __init__(
        self,
        on: float = 1.0,
        off: float = 0.0,
        threshold: float | None = None,
    ) -> None:
        self.on = on
        self.off = off
        self.threshold = threshold if threshold is not None else (on + off) / 2.0

    def to_policy(self, hardware_value: object) -> float:
        if isinstance(hardware_value, bool):
            return self.on if hardware_value else self.off
        return self.on if float(hardware_value) >= self.threshold else self.off  # type: ignore[arg-type]

    def to_hardware(self, policy_value: float) -> bool:
        return policy_value >= self.threshold


class ScaleMapping(Mapping):
    """Linear scale mapping between hardware and policy ranges.

    Parameters
    ----------
    hardware_min, hardware_max:
        Hardware value range.
    policy_min, policy_max:
        Policy value range.
    """

    def __init__(
        self,
        hardware_min: float = 0.0,
        hardware_max: float = 1.0,
        policy_min: float = 0.0,
        policy_max: float = 1.0,
    ) -> None:
        self.hardware_min = hardware_min
        self.hardware_max = hardware_max
        self.policy_min = policy_min
        self.policy_max = policy_max

    def to_policy(self, hardware_value: object) -> float:
        v = float(hardware_value)  # type: ignore[arg-type]
        hw_range = self.hardware_max - self.hardware_min
        if hw_range == 0:
            return self.policy_min
        normalized = (v - self.hardware_min) / hw_range
        return self.policy_min + normalized * (self.policy_max - self.policy_min)

    def to_hardware(self, policy_value: float) -> float:
        pol_range = self.policy_max - self.policy_min
        if pol_range == 0:
            return self.hardware_min
        normalized = (policy_value - self.policy_min) / pol_range
        return self.hardware_min + normalized * (self.hardware_max - self.hardware_min)


class EnumMapping(Mapping):
    """Map between hardware enum/string values and policy floats.

    Parameters
    ----------
    forward:
        hardware_value → policy_value mapping.
    """

    def __init__(self, forward: dict[str | bool, float]) -> None:
        self._forward = forward
        self._reverse = {v: k for k, v in forward.items()}

    def to_policy(self, hardware_value: object) -> float:
        return self._forward.get(hardware_value, 0.0)  # type: ignore[arg-type]

    def to_hardware(self, policy_value: float) -> bool | int | float | str:
        # Find the closest mapped value
        closest_key = min(self._reverse.keys(), key=lambda k: abs(k - policy_value))
        return self._reverse[closest_key]  # type: ignore[return-value]


# ---------------------------------------------------------------------------
# Observation entries
# ---------------------------------------------------------------------------


@dataclass
class _ObsJointPositions:
    """Observe joint positions from a motion group."""

    key: str
    source: MotionGroup | list[MotionGroup]
    writable: bool = True
    """If True (default), infer a matching joint action from this observation."""
    mode: str = "absolute"
    """Action mode for inferred actions: 'absolute' or 'relative'."""


@dataclass
class _ObsJointTorques:
    """Observe joint torques (read-only)."""

    key: str
    source: MotionGroup
    default: list[float] | None = None


@dataclass
class _ObsJointCurrents:
    """Observe joint currents (read-only)."""

    key: str
    source: MotionGroup
    default: list[float] | None = None


@dataclass
class _ObsTcp:
    """Observe TCP pose."""

    key: str
    source: MotionGroup
    tcp: str = ""
    format: TcpFormat = TcpFormat.ROTATION_VECTOR


@dataclass
class _ObsIO:
    """Observe an IO value."""

    key: str
    source: MotionGroup
    io: str
    mapping: Mapping = field(default_factory=IdentityMapping)
    writable: bool = True
    """If True (default), infer a matching IO action from this observation."""


@dataclass
class _ObsImage:
    """Observe a camera image."""

    key: str
    source: object
    """A CameraSource-compatible object (has connect/read/disconnect)."""


@dataclass
class _ObsConstant:
    """A constant value included in every observation."""

    key: str
    value: Any


@dataclass
class _ObsComputed:
    """Observation resolved by calling an async function at every inference step.

    The function receives the current flat observation dict (built so far)
    and returns a dict of additional key-value pairs to merge in.

    Use for external data sources like OPC UA, databases, HTTP APIs, etc.
    """

    fn: Any  # async (obs: dict) -> dict[str, Any]


ObservationEntry = (
    _ObsJointPositions
    | _ObsJointTorques
    | _ObsJointCurrents
    | _ObsTcp
    | _ObsIO
    | _ObsImage
    | _ObsConstant
    | _ObsComputed
)


class Observation:
    """Factory for observation entries. Each entry maps a policy key to a NOVA source."""

    @staticmethod
    def joint_positions(
        key: str,
        source: MotionGroup | list[MotionGroup],
        *,
        writable: bool = True,
        mode: str = "absolute",
    ) -> _ObsJointPositions:
        """Observe joint positions. Writable by default (infers a matching action)."""
        return _ObsJointPositions(key=key, source=source, writable=writable, mode=mode)

    @staticmethod
    def joint_torques(
        key: str,
        source: MotionGroup,
        *,
        default: list[float] | None = None,
    ) -> _ObsJointTorques:
        """Observe joint torques (read-only). Optional default if controller doesn't provide them."""
        return _ObsJointTorques(key=key, source=source, default=default)

    @staticmethod
    def joint_currents(
        key: str,
        source: MotionGroup,
        *,
        default: list[float] | None = None,
    ) -> _ObsJointCurrents:
        """Observe joint currents (read-only). Optional default if controller doesn't provide them."""
        return _ObsJointCurrents(key=key, source=source, default=default)

    @staticmethod
    def tcp(
        key: str,
        source: MotionGroup,
        *,
        tcp: str = "",
        format: TcpFormat = TcpFormat.ROTATION_VECTOR,
    ) -> _ObsTcp:
        """Observe TCP pose in the given representation format."""
        return _ObsTcp(key=key, source=source, tcp=tcp, format=format)

    @staticmethod
    def io(
        key: str,
        source: MotionGroup,
        io: str,
        *,
        mapping: Mapping | None = None,
        writable: bool = True,
    ) -> _ObsIO:
        """Observe an IO value (digital/analog input or output).

        Writable by default: the policy can return this key to write the IO.
        Set ``writable=False`` for read-only sensors.
        """
        return _ObsIO(
            key=key, source=source, io=io,
            mapping=mapping or IdentityMapping(), writable=writable,
        )

    @staticmethod
    def image(key: str, source: object) -> _ObsImage:
        """Observe a camera image.

        Args:
            key: Policy key for this camera (e.g. ``"exterior_image_1"``).
            source: A camera source object with ``connect()``, ``read()``,
                    ``disconnect()`` methods. Use a factory like
                    ``WebRTCCameras(...).device(id)`` to create one.
        """
        return _ObsImage(key=key, source=source)

    @staticmethod
    def constant(key: str, value: object) -> _ObsConstant:
        """A constant value included in every observation (e.g. language instruction)."""
        return _ObsConstant(key=key, value=value)

    @staticmethod
    def computed(fn: object) -> _ObsComputed:
        """Call an async function at every inference step to add custom observations.

        The function receives the current observation dict (joints, IOs, constants
        already filled in) and returns a dict of additional key-value pairs.

        Use for external data sources like OPC UA, databases, HTTP APIs, etc.

        Example::

            async def read_opcua(obs: dict) -> dict:
                values = await opcua_client.read_values(["ns=2;s=Temperature", "ns=2;s=Force"])
                return {"temperature": values[0], "force_z": values[1]}

            schema = PolicySchema(observations=[
                Observation.joint_positions("arm_joints", source=mg),
                Observation.computed(read_opcua),
            ])
        """
        return _ObsComputed(fn=fn)


# ---------------------------------------------------------------------------
# Action entries
# ---------------------------------------------------------------------------


@dataclass
class _ActJointPositions:
    """Explicit joint position action (when key differs from observation or multi-MG concat)."""

    key: str
    target: MotionGroup | list[MotionGroup]
    mode: str = "absolute"


@dataclass
class _ActIO:
    """IO write action."""

    key: str
    target: MotionGroup
    io: str
    mapping: Mapping = field(default_factory=IdentityMapping)


@dataclass
class _ActComputed:
    """Action resolved by calling an async function when the policy returns.

    The function receives the full action dict from the policy and can trigger
    arbitrary side effects (write to OPC UA, trigger PLC, send HTTP, etc.).
    """

    fn: Any  # async (action: dict) -> None


ActionEntry = _ActJointPositions | _ActIO | _ActComputed


class Action:
    """Factory for action entries."""

    @staticmethod
    def joint_positions(
        key: str,
        target: MotionGroup | list[MotionGroup],
        *,
        mode: str = "absolute",
    ) -> _ActJointPositions:
        """Explicit joint action (use when the action key differs from the observation key)."""
        return _ActJointPositions(key=key, target=target, mode=mode)

    @staticmethod
    def io(
        key: str,
        target: MotionGroup,
        io: str,
        *,
        mapping: Mapping | None = None,
    ) -> _ActIO:
        """IO write action (digital/analog output)."""
        return _ActIO(key=key, target=target, io=io, mapping=mapping or IdentityMapping())

    @staticmethod
    def computed(fn: object) -> _ActComputed:
        """Call an async function whenever the policy returns an action.

        The function receives the full action dict from the policy and can
        trigger arbitrary side effects — write to OPC UA, PLC, HTTP API, etc.

        Example::

            async def write_plc(action: dict) -> None:
                conveyor_speed = action.get("conveyor_speed", 0.0)
                await plc_client.write("ns=2;s=ConveyorSpeed", conveyor_speed)

            schema = PolicySchema(
                observations=[Observation.joint_positions("joints", source=mg)],
                actions=[Action.computed(write_plc)],
            )
        """
        return _ActComputed(fn=fn)


# ---------------------------------------------------------------------------
# Compiled schema internals (used by executor + clients, not user-facing)
# ---------------------------------------------------------------------------


@dataclass
class _JointMapping:
    """Internal: maps a policy joint key to one or more motion groups."""

    key: str
    sources: list[MotionGroup]
    writable: bool
    mode: str  # "absolute" or "relative"


@dataclass
class _IOMapping:
    """Internal: maps a policy IO key to hardware."""

    key: str
    motion_group: MotionGroup
    hardware_key: str
    mapping: Mapping


@dataclass
class _TcpMapping:
    """Internal: maps a policy TCP key to a motion group + format."""

    key: str
    source: MotionGroup
    tcp: str
    format: TcpFormat


@dataclass
class GroupedObservation:
    """Per-motion-group extracted observation data.

    Produced by ``PolicySchema.build_grouped_observation()`` and consumed by
    policy clients to build their transport-specific formats.
    """

    motion_group_id: str
    key: str
    joints: list[float]
    tcp: list[float] | None = None
    tcp_key: str = ""
    ios: dict[str, float] | None = None
    """IO policy-key → policy-float value."""


# ---------------------------------------------------------------------------
# PolicySchema
# ---------------------------------------------------------------------------


class PolicySchema:
    """Declares what a policy observes and controls.

    Validates eagerly at construction time. Compiles observations and actions
    into internal lookup structures used by the executor and policy clients.
    """

    def __init__(
        self,
        observations: list[ObservationEntry],
        actions: list[ActionEntry] | None = None,
    ) -> None:
        self._observations = list(observations)
        self._actions = list(actions or [])

        # Compiled structures
        self._joint_mappings: list[_JointMapping] = []
        self._joint_action_mappings: list[_JointMapping] = []
        self._tcp_mappings: list[_TcpMapping] = []
        self._obs_io_mappings: list[_IOMapping] = []
        self._act_io_mappings: list[_IOMapping] = []
        self._constants: dict[str, Any] = {}
        self._image_sources: dict[str, object] = {}
        self._computed_obs_fns: list[Any] = []  # async (obs) -> dict
        self._computed_act_fns: list[Any] = []  # async (action) -> None

        self._compile()
        self._validate()

    @property
    def observations(self) -> list[ObservationEntry]:
        return self._observations

    @property
    def actions(self) -> list[ActionEntry]:
        return self._actions

    def _compile(self) -> None:
        """Compile observation/action entries into internal mappings."""
        explicit_action_keys = self._compile_actions()
        self._compile_observations(explicit_action_keys)

    def _compile_actions(self) -> set[str]:
        """Compile explicit action entries. Returns their keys."""
        keys: set[str] = set()
        for act in self._actions:
            if isinstance(act, _ActJointPositions):
                targets = act.target if isinstance(act.target, list) else [act.target]
                self._joint_action_mappings.append(
                    _JointMapping(key=act.key, sources=targets, writable=True, mode=act.mode)
                )
                keys.add(act.key)
            elif isinstance(act, _ActIO):
                self._act_io_mappings.append(
                    _IOMapping(
                        key=act.key, motion_group=act.target,
                        hardware_key=act.io, mapping=act.mapping,
                    )
                )
                keys.add(act.key)
            elif isinstance(act, _ActComputed):
                self._computed_act_fns.append(act.fn)
        return keys

    def _compile_observations(self, explicit_action_keys: set[str]) -> None:
        """Compile observation entries into internal mappings."""
        for obs in self._observations:
            if isinstance(obs, _ObsJointPositions):
                sources = obs.source if isinstance(obs.source, list) else [obs.source]
                mapping = _JointMapping(
                    key=obs.key, sources=sources, writable=obs.writable, mode=obs.mode,
                )
                self._joint_mappings.append(mapping)
                if obs.writable and obs.key not in explicit_action_keys:
                    self._joint_action_mappings.append(mapping)
            elif isinstance(obs, _ObsTcp):
                tcp_name = obs.tcp
                self._tcp_mappings.append(
                    _TcpMapping(key=obs.key, source=obs.source, tcp=tcp_name, format=obs.format)
                )
            elif isinstance(obs, _ObsIO):
                self._obs_io_mappings.append(
                    _IOMapping(
                        key=obs.key, motion_group=obs.source,
                        hardware_key=obs.io, mapping=obs.mapping,
                    )
                )
                # Infer IO action if writable and no explicit action exists
                if obs.writable and obs.key not in explicit_action_keys:
                    self._act_io_mappings.append(
                        _IOMapping(
                            key=obs.key, motion_group=obs.source,
                            hardware_key=obs.io, mapping=obs.mapping,
                        )
                    )
            elif isinstance(obs, _ObsImage):
                self._image_sources[obs.key] = obs.source
            elif isinstance(obs, _ObsConstant):
                self._constants[obs.key] = obs.value
            elif isinstance(obs, _ObsComputed):
                self._computed_obs_fns.append(obs.fn)

    def _validate(self) -> None:
        """Validate schema consistency."""
        # Check for duplicate keys
        all_obs_keys = [self._obs_key(o) for o in self._observations]
        seen: set[str] = set()
        for k in all_obs_keys:
            if k in seen:
                msg = f"Duplicate observation key: {k!r}"
                raise ValueError(msg)
            seen.add(k)

        all_act_keys = [self._act_key(a) for a in self._actions]
        act_seen: set[str] = set()
        for k in all_act_keys:
            if k in act_seen:
                msg = f"Duplicate action key: {k!r}"
                raise ValueError(msg)
            act_seen.add(k)

    @staticmethod
    def _obs_key(entry: ObservationEntry) -> str:
        if isinstance(entry, _ObsComputed):
            return f"__computed_{id(entry)}__"
        return entry.key

    @staticmethod
    def _act_key(entry: ActionEntry) -> str:
        if isinstance(entry, _ActComputed):
            return f"__computed_{id(entry)}__"
        return entry.key

    # ------------------------------------------------------------------
    # Motion groups
    # ------------------------------------------------------------------

    def get_motion_groups(self) -> list[MotionGroup]:
        """All unique motion groups referenced by observations and actions."""
        seen_ids: set[str] = set()
        result: list[MotionGroup] = []
        for m in self._joint_mappings:
            for mg in m.sources:
                if mg.id not in seen_ids:
                    seen_ids.add(mg.id)
                    result.append(mg)
        for m in self._tcp_mappings:
            if m.source.id not in seen_ids:
                seen_ids.add(m.source.id)
                result.append(m.source)
        for m in self._obs_io_mappings:
            if m.motion_group.id not in seen_ids:
                seen_ids.add(m.motion_group.id)
                result.append(m.motion_group)
        for m in self._act_io_mappings:
            if m.motion_group.id not in seen_ids:
                seen_ids.add(m.motion_group.id)
                result.append(m.motion_group)
        return result

    @property
    def image_sources(self) -> dict[str, object]:
        """Policy key -> camera source for all image observations."""
        return dict(self._image_sources)

    @property
    def tcp(self) -> str:
        """TCP name from the first TCP observation (or empty for default)."""
        for m in self._tcp_mappings:
            if m.tcp:
                return m.tcp
        return ""

    def io_keys_by_controller(self) -> dict[str, list[str]]:
        """Hardware IO keys grouped by controller ID (for IO streaming)."""
        result: dict[str, set[str]] = {}
        for m in [*self._obs_io_mappings, *self._act_io_mappings]:
            ctrl = get_controller_id(m.motion_group)
            result.setdefault(ctrl, set()).add(m.hardware_key)
        return {k: sorted(v) for k, v in result.items() if v}

    # ------------------------------------------------------------------
    # Observation building
    # ------------------------------------------------------------------

    async def build_observation(
        self,
        states: dict[str, RobotState],
        io_values: dict[str, object] | None = None,
    ) -> dict[str, Any]:
        """Build a flat feature dict from current robot states.

        Used by CallbackPolicyClient.
        """
        obs: dict[str, Any] = {}
        self._build_obs_joints(obs, states)
        self._build_obs_tcp(obs, states)
        self._build_obs_optional_joints(obs, states)
        self._build_obs_ios(obs, io_values)
        obs.update(self._constants)
        for fn in self._computed_obs_fns:
            extra = await fn(obs)
            if isinstance(extra, dict):
                obs.update(extra)
        return obs

    def _build_obs_joints(self, obs: dict[str, float], states: dict[str, RobotState]) -> None:
        for m in self._joint_mappings:
            all_joints: list[float] = []
            for mg in m.sources:
                state = states.get(mg.id)
                if state is not None:
                    all_joints.extend(state.joints)
            for i, v in enumerate(all_joints, start=1):
                obs[f"{m.key}_{i}"] = v

    def _build_obs_tcp(self, obs: dict[str, float], states: dict[str, RobotState]) -> None:
        for m in self._tcp_mappings:
            state = states.get(m.source.id)
            if state is not None and hasattr(state, "pose") and state.pose is not None:
                tcp_vals = pose_to_tcp(state.pose, m.format)
                for i, v in enumerate(tcp_vals, start=1):
                    obs[f"{m.key}_{i}"] = v

    def _build_obs_optional_joints(
        self, obs: dict[str, float], states: dict[str, RobotState],
    ) -> None:
        for entry in self._observations:
            if isinstance(entry, _ObsJointTorques):
                state = states.get(entry.source.id)
                values = getattr(state, "joint_torques", None) if state is not None else None
                self._fill_optional_array(obs, entry.key, values, entry.default)
            elif isinstance(entry, _ObsJointCurrents):
                state = states.get(entry.source.id)
                values = getattr(state, "joint_currents", None) if state is not None else None
                self._fill_optional_array(obs, entry.key, values, entry.default)

    @staticmethod
    def _fill_optional_array(
        obs: dict[str, float],
        key: str,
        values: tuple[float, ...] | None,
        default: list[float] | None,
    ) -> None:
        data = values or default
        if data is not None:
            for i, v in enumerate(data, start=1):
                obs[f"{key}_{i}"] = v

    def _build_obs_ios(
        self, obs: dict[str, float], io_values: dict[str, object] | None,
    ) -> None:
        if io_values is None:
            return
        for m in self._obs_io_mappings:
            raw = io_values.get(m.hardware_key)
            obs[m.key] = m.mapping.to_policy(raw) if raw is not None else 0.0

    def build_grouped_observation(
        self,
        states: dict[str, RobotState],
        io_values: dict[str, object] | None = None,
    ) -> list[GroupedObservation]:
        """Build per-motion-group observations (used by GR00T client)."""
        result: list[GroupedObservation] = []
        seen_mg: set[str] = set()
        for m in self._joint_mappings:
            for mg in m.sources:
                if mg.id in seen_mg:
                    continue
                seen_mg.add(mg.id)
                state = states.get(mg.id)
                if state is None:
                    continue
                result.append(self._build_group_obs(mg, m.key, state, io_values))
        return result

    def _build_group_obs(
        self,
        mg: MotionGroup,
        key: str,
        state: RobotState,
        io_values: dict[str, object] | None,
    ) -> GroupedObservation:
        """Build a single GroupedObservation for one motion group."""
        tcp_vals, tcp_key = self._find_tcp_for_mg(mg, state)
        io_floats = self._find_ios_for_mg(mg, io_values)
        return GroupedObservation(
            motion_group_id=mg.id, key=key, joints=list(state.joints),
            tcp=tcp_vals, tcp_key=tcp_key, ios=io_floats,
        )

    def _find_tcp_for_mg(
        self, mg: MotionGroup, state: RobotState,
    ) -> tuple[list[float] | None, str]:
        for tm in self._tcp_mappings:
            if tm.source.id == mg.id and hasattr(state, "pose") and state.pose is not None:
                return pose_to_tcp(state.pose, tm.format), tm.key
        return None, ""

    def _find_ios_for_mg(
        self, mg: MotionGroup, io_values: dict[str, object] | None,
    ) -> dict[str, float] | None:
        if io_values is None:
            return None
        result: dict[str, float] | None = None
        for iom in self._obs_io_mappings:
            if iom.motion_group.id == mg.id:
                if result is None:
                    result = {}
                raw = io_values.get(iom.hardware_key)
                result[iom.key] = iom.mapping.to_policy(raw) if raw is not None else 0.0
        return result

    # ------------------------------------------------------------------
    # Action parsing
    # ------------------------------------------------------------------

    async def parse_action(self, action: dict[str, float]) -> tuple[
        dict[str, list[list[float]]],
        dict[str, dict[str, bool | int | float | str]] | None,
    ]:
        """Parse a flat action dict into per-MG joints and IOs."""
        joints: dict[str, list[list[float]]] = {}
        ios: dict[str, dict[str, bool | int | float | str]] = {}

        # Joint actions
        for m in self._joint_action_mappings:
            all_values: list[float] = []
            idx = 1
            while f"{m.key}_{idx}" in action:
                all_values.append(float(action[f"{m.key}_{idx}"]))
                idx += 1
            if not all_values:
                continue

            if len(m.sources) == 1:
                joints[m.sources[0].id] = [all_values]
            else:
                per_mg = len(all_values) // len(m.sources)
                for i, mg in enumerate(m.sources):
                    mg_vals = all_values[i * per_mg : (i + 1) * per_mg]
                    if mg_vals:
                        joints[mg.id] = [mg_vals]

        # IO actions
        for m in self._act_io_mappings:
            if m.key not in action:
                continue
            hw_value = m.mapping.to_hardware(float(action[m.key]))
            ios.setdefault(m.motion_group.id, {})[m.hardware_key] = hw_value

        # Computed action side effects
        for fn in self._computed_act_fns:
            await fn(action)

        return joints, ios or None
