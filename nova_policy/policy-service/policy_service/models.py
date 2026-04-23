from __future__ import annotations

from enum import StrEnum

from pydantic import BaseModel, ConfigDict, Field

JsonPrimitive = str | int | float | bool | None
JsonValue = JsonPrimitive | dict[str, object] | list[object]


class AppState(StrEnum):
    EMPTY = "EMPTY"
    LOADING = "LOADING"
    READY = "READY"
    RUNNING = "RUNNING"
    SWITCHING = "SWITCHING"
    ERROR = "ERROR"


class RunState(StrEnum):
    PREPARING = "PREPARING"
    RUNNING = "RUNNING"
    STOPPING = "STOPPING"
    STOPPED = "STOPPED"
    TIMED_OUT = "TIMED_OUT"
    FAILED = "FAILED"


class PolicyStartRequest(BaseModel):
    model_config = ConfigDict(extra="allow")

    task: str | None = None
    timeout_s: float = Field(default=120.0, gt=0)
    policy: dict[str, JsonValue] | None = None
    target: dict[str, JsonValue] | None = None
    cameras: dict[str, JsonValue] | None = None
    gripper: dict[str, JsonValue] | None = None
    motion_group_setup: dict[str, JsonValue] | None = None


class PolicyRunResponse(BaseModel):
    run: str
    policy: str
    state: RunState
    start_time: str
    timeout_s: float
    elapsed_s: float | None = None
    metadata: dict[str, JsonValue] | None = None


class PolicyInfoResponse(BaseModel):
    policy: str
    loaded: bool
    app_state: AppState


class HealthzResponse(BaseModel):
    status: str
