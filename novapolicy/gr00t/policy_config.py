"""Declared deployment config of a GR00T policy (``policy.toml``).

Nothing here is guessed or fitted: every value is written down by whoever deploys the
policy, from the training setup and the cell. ``load`` rejects missing values, unknown keys,
unsupported options and ``"TODO"`` placeholders.
``python -m novapolicy.gr00t.inspect_policy --config policy.toml`` checks the declared values
against the server, the checkpoint, the training data and the robot.

Sections (a complete file: ``examples/gr00t_single_arm_pick_and_place/policy.toml``)::

    [server]               host, port of the GR00T inference server
    [policy]               prompt, language_key, rate_hz, action_horizon, state/action/video
                           keys, image_width, image_height (the training camera resolution)
    [policy.joints]        key, unit ("rad" | "deg")
    [policy.tcp_action]    key, representation ("delta": per-step change applied to the pose
                           the previous step reached), frame ("base" | "tool"),
                           base_rotation_z_deg, position_unit ("m" | "mm"),
                           rotation_format ("rotvec"), gain_position, gain_rotation (realized
                           / commanded motion per step in training; 1 for plain targets)
    [policy.gripper]       state_key, action_key, action ("binary"), close_when
                           ("negative" | "positive"), state_open, state_closed, state_holding
    [cell]                 controller, tcp (the point the end-effector values refer to),
                           home_deg, camera_server, gripper_host, gripper_open_gap_mm
    [cell.cameras]         video key -> camera serial number
    [guards]               max_tcp_speed_mm_s, max_chunk_travel_mm, max_rotation_step_deg,
                           max_tilt_deg, max_yaw_deg (vs. the run's start heading; 180 = no
                           limit), workspace_x_mm, workspace_y_mm ([min, max]), min_tcp_z_mm
"""

from __future__ import annotations

from dataclasses import dataclass, fields
from pathlib import Path
import tomllib
from typing import Any, TypeVar, cast, get_type_hints

TODO = "TODO"

_T = TypeVar("_T")
_PAIR = 2  # [min, max]
_CHOICES: dict[str, tuple[str, ...]] = {
    "unit": ("rad", "deg"),
    "representation": ("delta",),
    "frame": ("base", "tool"),
    "position_unit": ("m", "mm"),
    "rotation_format": ("rotvec",),
    "action": ("binary",),
    "close_when": ("negative", "positive"),
}


class PolicyConfigError(ValueError):
    """A ``policy.toml`` that is incomplete, unknown or unsupported."""


@dataclass(frozen=True)
class Server:
    host: str
    port: int


@dataclass(frozen=True)
class Joints:
    key: str
    unit: str


@dataclass(frozen=True)
class TcpAction:
    key: str
    representation: str
    frame: str
    base_rotation_z_deg: float
    position_unit: str
    rotation_format: str
    gain_position: float
    gain_rotation: float

    @property
    def position_scale_mm(self) -> float:
        """Policy position unit -> mm."""
        return 1000.0 if self.position_unit == "m" else 1.0


@dataclass(frozen=True)
class Gripper:
    state_key: str
    action_key: str
    action: str
    close_when: str
    state_open: float
    state_closed: float
    state_holding: float

    def closes(self, value: float) -> bool:
        """Whether a binary gripper action asks to close."""
        return value < 0.0 if self.close_when == "negative" else value > 0.0


@dataclass(frozen=True)
class Policy:
    prompt: str
    language_key: str
    rate_hz: float
    action_horizon: int
    state_keys: list[str]
    action_keys: list[str]
    video_keys: list[str]
    image_width: int
    image_height: int
    joints: Joints
    tcp_action: TcpAction
    gripper: Gripper

    @property
    def dt_ms(self) -> float:
        return 1000.0 / self.rate_hz


@dataclass(frozen=True)
class Cell:
    controller: str
    tcp: str
    home_deg: list[float]
    camera_server: str
    gripper_host: str
    gripper_open_gap_mm: float  # the real gripper's fully open jaw gap
    cameras: dict[str, str]


@dataclass(frozen=True)
class Guards:
    max_tcp_speed_mm_s: float
    max_chunk_travel_mm: float
    max_rotation_step_deg: float
    max_tilt_deg: float
    max_yaw_deg: float
    workspace_x_mm: list[float]
    workspace_y_mm: list[float]
    min_tcp_z_mm: float


@dataclass(frozen=True)
class PolicyConfig:
    server: Server
    policy: Policy
    cell: Cell
    guards: Guards


def _build(cls: type[_T], data: object, where: str) -> _T:
    """Strictly build dataclass ``cls`` from a TOML table."""
    if not isinstance(data, dict):
        msg = f"[{where}] must be a table"
        raise PolicyConfigError(msg)
    table = cast("dict[str, Any]", data)
    hints = get_type_hints(cls)
    names = {f.name for f in fields(cast("Any", cls))}
    if unknown := sorted(set(table) - names):
        msg = f"[{where}] has unknown keys {unknown}"
        raise PolicyConfigError(msg)
    if missing := sorted(names - set(table)):
        msg = f"[{where}] is missing {missing}"
        raise PolicyConfigError(msg)
    values: dict[str, Any] = {}
    for name in sorted(names):
        values[name] = _value(hints[name], table[name], f"{where}.{name}")
    return cls(**values)


def _value(hint: object, raw: object, where: str) -> object:
    if raw == TODO:
        msg = f"{where} is still TODO: declare it"
        raise PolicyConfigError(msg)
    if isinstance(hint, type) and hasattr(hint, "__dataclass_fields__"):
        return _build(cast("type[Any]", hint), raw, where)
    origin = getattr(hint, "__origin__", None)
    if origin is list:
        if not isinstance(raw, list):
            msg = f"{where} must be a list"
            raise PolicyConfigError(msg)
        (item,) = cast("Any", hint).__args__
        return [_value(item, v, f"{where}[{i}]") for i, v in enumerate(raw)]
    if origin is dict:
        if not isinstance(raw, dict):
            msg = f"{where} must be a table"
            raise PolicyConfigError(msg)
        return {str(k): _value(str, v, f"{where}.{k}") for k, v in raw.items()}
    return _scalar(hint, raw, where)


def _scalar(hint: object, raw: object, where: str) -> object:
    if hint is float and isinstance(raw, int | float) and not isinstance(raw, bool):
        return float(raw)
    if hint is int and isinstance(raw, int) and not isinstance(raw, bool):
        return raw
    if hint is str and isinstance(raw, str):
        name = where.rsplit(".", 1)[-1]
        if name in _CHOICES and raw not in _CHOICES[name]:
            msg = f"{where} = {raw!r}: supported values are {list(_CHOICES[name])}"
            raise PolicyConfigError(msg)
        return raw
    msg = f"{where} = {raw!r} must be {getattr(hint, '__name__', hint)}"
    raise PolicyConfigError(msg)


def _check(config: PolicyConfig) -> None:
    """Cross-field consistency that types alone cannot express."""
    p, c, g = config.policy, config.cell, config.guards
    keys = set(p.state_keys) | set(p.action_keys)
    for name, key in (
        ("policy.joints.key", p.joints.key),
        ("policy.tcp_action.key", p.tcp_action.key),
        ("policy.gripper.state_key", p.gripper.state_key),
        ("policy.gripper.action_key", p.gripper.action_key),
    ):
        if key not in keys:
            msg = f"{name} = {key!r} is not among the declared state/action keys"
            raise PolicyConfigError(msg)
    if sorted(c.cameras) != sorted(p.video_keys):
        msg = f"cell.cameras {sorted(c.cameras)} must map exactly the video keys {p.video_keys}"
        raise PolicyConfigError(msg)
    for name, pair in (("workspace_x_mm", g.workspace_x_mm), ("workspace_y_mm", g.workspace_y_mm)):
        if len(pair) != _PAIR or pair[0] >= pair[1]:
            msg = f"guards.{name} must be [min, max]"
            raise PolicyConfigError(msg)
    if p.rate_hz <= 0 or p.action_horizon <= 0:
        msg = "policy.rate_hz and policy.action_horizon must be positive"
        raise PolicyConfigError(msg)


def load(path: str | Path) -> PolicyConfig:
    """Read and strictly validate a ``policy.toml``."""
    path = Path(path)
    try:
        data = tomllib.loads(path.read_text())
    except tomllib.TOMLDecodeError as e:
        msg = f"{path}: {e}"
        raise PolicyConfigError(msg) from e
    try:
        config = _build(PolicyConfig, data, "root")
        _check(config)
    except PolicyConfigError as e:
        msg = f"{path}: {e}"
        raise PolicyConfigError(msg) from e
    return config
