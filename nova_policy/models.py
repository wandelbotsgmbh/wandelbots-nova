from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal, Required, TypeAlias, TypedDict

JsonPrimitive: TypeAlias = str | int | float | bool | None
JsonValue: TypeAlias = JsonPrimitive | dict[str, "JsonValue"] | list["JsonValue"]


class PolicyRunPayload(TypedDict, total=False):
    run: Required[str]
    policy: str
    state: Required[str]
    start_time: str
    timeout_s: float
    elapsed_s: float
    metadata: dict[str, JsonValue]


class PolicyDescriptorPayload(TypedDict):
    kind: Required[str]
    path: Required[str]


@dataclass(slots=True, frozen=True)
class ACTPolicy:
    path: str
    n_action_steps: int | None = None

    @property
    def kind(self) -> Literal["act"]:
        return "act"


PolicySpec: TypeAlias = ACTPolicy


@dataclass(slots=True)
class PolicyRun:
    run: str
    policy: str
    state: str
    start_time: str | None = None
    timeout_s: float | None = None
    elapsed_s: float | None = None
    metadata: dict[str, JsonValue] | None = None

    @classmethod
    def from_dict(cls, data: PolicyRunPayload) -> PolicyRun:
        return cls(
            run=data["run"],
            policy=data.get("policy", ""),
            state=data["state"],
            start_time=data.get("start_time"),
            timeout_s=data.get("timeout_s"),
            elapsed_s=data.get("elapsed_s"),
            metadata=data.get("metadata"),
        )


@dataclass(slots=True, frozen=True)
class RobotStatePoint:
    joints: dict[str, float]
    gripper: dict[str, float] | None = None
    tcp: dict[str, float] | None = None
    io: dict[str, JsonPrimitive] | None = None
    timestamp_s: float | None = None

    def to_observation(self) -> dict[str, JsonValue]:
        observation: dict[str, JsonValue] = dict(self.joints)
        if self.gripper is not None:
            observation.update(self.gripper)
        if self.tcp is not None:
            observation.update(self.tcp)
        if self.io is not None:
            observation.update(self.io)
        if self.timestamp_s is not None:
            observation["timestamp"] = self.timestamp_s
        return observation


@dataclass(slots=True, frozen=True)
class ActionStep:
    joints: dict[str, float] = field(default_factory=dict)
    gripper: dict[str, float] | None = None
    io: dict[str, JsonPrimitive] | None = None


@dataclass(slots=True, frozen=True)
class ActionChunk:
    run: str
    policy: str
    policy_kind: str
    chunk_id: str
    observation_seq: int
    n_action_steps: int
    control_dt_s: float
    inference_latency_ms: float
    steps: list[ActionStep]
    model_time: float | None = None
    first_step_at_s: float | None = None
    diagnostics: dict[str, JsonValue] | None = None
