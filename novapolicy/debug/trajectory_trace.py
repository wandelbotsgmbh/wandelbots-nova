"""Optional trajectory diagnostics kept out of policy control classes."""

from __future__ import annotations

from dataclasses import dataclass, field
import json
from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from collections.abc import Sequence

    import numpy as np

    from nova.types import Pose, RobotState
    from novapolicy.policy_client import PolicyClient
    from novapolicy.types import ActionChunk, JoggingMode


@dataclass(slots=True)
class RawActionChunkTrace:
    """Raw timestamped predictions received by an asynchronous policy client."""

    chunks: list[dict[str, object]] = field(default_factory=list)

    def clear(self) -> None:
        self.chunks.clear()

    def record(self, actions: Sequence[tuple[int, np.ndarray]]) -> None:
        self.chunks.append({
            "first_timestep": actions[0][0],
            "actions": [
                {"timestep": timestep, "values": action.tolist()} for timestep, action in actions
            ],
        })

    @property
    def data(self) -> dict[str, object]:
        return {"raw_action_chunks": self.chunks}


@dataclass(slots=True)
class WaypointTrajectoryTrace:
    """Controller samples and waypoint requests for one motion group."""

    motion_group_id: str
    mode: JoggingMode
    states: list[dict[str, object]] = field(default_factory=list)
    requests: list[dict[str, object]] = field(default_factory=list)

    def record_state(
        self,
        *,
        server_timestamp_ms: int,
        joints: list[float],
        tcp: Pose | None,
    ) -> None:
        self.states.append({
            "server_timestamp_ms": server_timestamp_ms,
            "joints": joints,
            "tcp": [*tcp.position, *tcp.orientation] if tcp is not None else None,
        })

    def record_request(
        self,
        *,
        sequence: int,
        action_timestep: int,
        policy_dt_ms: float,
        first_timestamp_ms: int | None,
        timestamp_offset_steps: int,
        server_dt_ms: float | None,
        server_sample_ms: int,
        timestamps_ms: list[int],
        steps: list[list[float]],
    ) -> None:
        self.requests.append({
            "sequence": sequence,
            "action_timestep": action_timestep,
            "policy_dt_ms": policy_dt_ms,
            "first_timestamp_ms": first_timestamp_ms,
            "timestamp_offset_steps": timestamp_offset_steps,
            "server_dt_ms": server_dt_ms,
            "server_sample_ms": server_sample_ms,
            "timestamps_ms": timestamps_ms,
            "steps": steps,
        })

    @property
    def data(self) -> dict[str, object]:
        return {
            "motion_group_id": self.motion_group_id,
            "mode": self.mode,
            "states": self.states,
            "requests": self.requests,
        }


class ExecutionTrajectoryTrace:
    """Episode-level trace recorder that writes only after execution stops."""

    def __init__(self, path: str | Path) -> None:
        self._path = Path(path)
        self._policy_chunks: list[dict[str, object]] = []
        self._sessions: dict[str, WaypointTrajectoryTrace] = {}

    def clear(self) -> None:
        self._policy_chunks.clear()
        self._sessions.clear()

    @staticmethod
    def enable_policy_client(policy: PolicyClient) -> None:
        policy.enable_trajectory_trace()

    def create_session_trace(
        self,
        motion_group_id: str,
        mode: JoggingMode,
    ) -> WaypointTrajectoryTrace:
        trace = WaypointTrajectoryTrace(motion_group_id, mode)
        self._sessions[motion_group_id] = trace
        return trace

    def record_policy_chunk(
        self,
        action: ActionChunk,
        robot_states: dict[str, RobotState],
        server_timestamps_ms: dict[str, int],
        *,
        step: int,
    ) -> None:
        self._policy_chunks.append({
            "step": step,
            "action_timestep": action.action_timestep,
            "dt_ms": action.dt_ms,
            "joints": action.joints,
            "tcp": action.tcp,
            "controller_samples": {
                group_id: {
                    "server_timestamp_ms": server_timestamp,
                    "joints": (
                        list(state.joints)
                        if (state := robot_states.get(group_id)) is not None
                        else None
                    ),
                }
                for group_id, server_timestamp in server_timestamps_ms.items()
            },
        })

    def write(
        self,
        *,
        reason: str | None,
        steps: int | None,
        duration_s: float | None,
        policy: PolicyClient,
    ) -> None:
        policy_trace = policy.trajectory_trace
        payload = {
            "format_version": 3,
            "result": (
                {"reason": reason, "steps": steps, "duration_s": duration_s}
                if reason is not None
                else None
            ),
            "policy_chunks": self._policy_chunks,
            "policy_client": policy_trace,
            "sessions": {
                group_id: session_trace.data for group_id, session_trace in self._sessions.items()
            },
        }
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._path.write_text(json.dumps(payload, separators=(",", ":")))

    @property
    def path(self) -> Path:
        return self._path
