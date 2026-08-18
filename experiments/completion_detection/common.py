"""Shared plumbing for the completion-detection experiments.

Companion to docs/architecture/incoming/execution-completion-detection.md and
…-research.md. Everything here is deliberately small and dependency-free beyond
what the SDK already ships.
"""

from __future__ import annotations

import asyncio
import json
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from decouple import config as env_config

DATA_DIR = Path(__file__).parent / "data"

# Categories a received MotionGroupState frame can fall into (wire-level).
CAT_NO_EXECUTE = "no_execute"
CAT_RUNNING = "RUNNING"
CAT_ENDED = "END_OF_TRAJECTORY"
CAT_PAUSED_USER = "PAUSED_BY_USER"
CAT_WAIT_IO = "WAIT_FOR_IO"
CAT_PAUSED_IO = "PAUSED_ON_IO"
CAT_EXECUTE_OTHER = "execute_other"  # execute set but no TRAJECTORY details

FRAME_CATEGORIES = [
    CAT_NO_EXECUTE,
    CAT_RUNNING,
    CAT_ENDED,
    CAT_PAUSED_USER,
    CAT_WAIT_IO,
    CAT_PAUSED_IO,
    CAT_EXECUTE_OTHER,
]


def nova_host() -> str:
    host = env_config("NOVA_API")
    return str(host).rstrip("/")


def cell_name() -> str:
    return str(env_config("CELL_NAME", default="cell"))


def access_token() -> str | None:
    token = env_config("NOVA_ACCESS_TOKEN", default=None)
    return str(token) if token else None


def ws_base() -> str:
    """The websocket base URL for the v2 API, mirroring the generated client's logic."""
    host = nova_host()
    if host.startswith("https://"):
        return "wss://" + host[len("https://") :] + "/api/v2"
    return "ws://" + host[len("http://") :] + "/api/v2"


def categorize_frame(state: dict[str, Any]) -> str:
    """Wire-level category of a raw MotionGroupState dict (the ``result`` object)."""
    execute = state.get("execute")
    if not execute:
        return CAT_NO_EXECUTE
    details = execute.get("details") or {}
    if details.get("kind") != "TRAJECTORY":
        return CAT_EXECUTE_OTHER
    return (details.get("state") or {}).get("kind", CAT_EXECUTE_OTHER)


def frame_location(state: dict[str, Any]) -> float | None:
    execute = state.get("execute")
    if not execute:
        return None
    details = execute.get("details") or {}
    if details.get("kind") != "TRAJECTORY":
        return None
    return details.get("location")


@dataclass
class EventLog:
    """Wall-clock-stamped experiment events (init sent, ack received, pause, …)."""

    events: list[dict[str, Any]] = field(default_factory=list)

    def emit(self, name: str, **payload: Any) -> None:
        self.events.append({"t_ns": time.time_ns(), "event": name, **payload})

    def write(self, path: Path) -> None:
        with path.open("w") as f:
            for event in self.events:
                f.write(json.dumps(event, default=str) + "\n")


def write_jsonl(path: Path, records: list[dict[str, Any]]) -> None:
    with path.open("w") as f:
        for record in records:
            f.write(json.dumps(record, default=str) + "\n")


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    with path.open() as f:
        return [json.loads(line) for line in f if line.strip()]


def parse_rate(spec: str) -> int | None:
    """``step`` → None (no response_rate param → controller step rate), else int ms."""
    if spec.strip().lower() == "step":
        return None
    return int(spec)


def rate_label(rate: int | None) -> str:
    return "step" if rate is None else f"{rate}ms"


async def cancel_and_wait(*tasks: asyncio.Task) -> None:
    for task in tasks:
        if not task.done():
            task.cancel()
    await asyncio.gather(*tasks, return_exceptions=True)
