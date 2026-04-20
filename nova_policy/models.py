from __future__ import annotations

from dataclasses import dataclass
from typing import Required, TypeAlias, TypedDict

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
