"""Lean observers for the MotionGroupState stream and the pull endpoint.

The websocket observer connects directly with ``websockets`` (same library the
generated client uses) and does nothing in the hot loop except append the raw
frame with a receive timestamp — no pydantic, no json parsing, no disk I/O.
Frames are parsed and written to JSONL only after the run ends.
"""

from __future__ import annotations

import asyncio
import json
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import websockets

from experiments.completion_detection.common import access_token, rate_label, ws_base


@dataclass
class StateStreamObserver:
    """Records every frame received on one motion-group state-stream websocket."""

    cell: str
    controller: str
    motion_group: str
    response_rate: int | None  # None → omit the parameter (controller step rate)
    started: asyncio.Event = field(default_factory=asyncio.Event)
    # (t_wall_ns, t_mono_ns, raw_text)
    _frames: list[tuple[int, int, str]] = field(default_factory=list)
    _error: str | None = None

    @property
    def label(self) -> str:
        return rate_label(self.response_rate)

    def _url(self) -> str:
        url = (
            f"{ws_base()}/cells/{self.cell}/controllers/{self.controller}"
            f"/motion-groups/{self.motion_group}/state-stream"
        )
        if self.response_rate is not None:
            url += f"?response_rate={self.response_rate}"
        return url

    async def run(self) -> None:
        """Consume the stream until cancelled. Never applies client-side backpressure."""
        headers: list[tuple[str, str]] = []
        token = access_token()
        if token:
            headers.append(("Authorization", f"Bearer {token}"))
        try:
            async with websockets.connect(
                self._url(), open_timeout=10, additional_headers=headers, max_queue=None
            ) as ws:
                self.started.set()
                frames = self._frames
                async for message in ws:
                    frames.append((time.time_ns(), time.monotonic_ns(), str(message)))
        except asyncio.CancelledError:
            raise
        except Exception as e:  # recorded, not raised — one dead observer must not kill the run
            self._error = f"{type(e).__name__}: {e}"
            self.started.set()

    def write(self, path: Path) -> dict[str, Any]:
        """Parse buffered frames and write JSONL. Returns a small stats dict."""
        n_bad = 0
        with path.open("w") as f:
            for t_ns, mono_ns, raw in self._frames:
                try:
                    state = json.loads(raw)["result"]
                except Exception:
                    n_bad += 1
                    state = {"_unparseable": raw[:500]}
                f.write(json.dumps({"t_ns": t_ns, "mono_ns": mono_ns, "state": state}) + "\n")
        return {
            "rate": self.label,
            "frames": len(self._frames),
            "unparseable": n_bad,
            "error": self._error,
        }


@dataclass
class StatePoller:
    """Polls GET …/motion-groups/{mg}/state at a low rate (briefing §7.5 / research §10.5)."""

    motion_group_api: Any  # wandelbots_api_client.v2 MotionGroupApi
    cell: str
    controller: str
    motion_group: str
    interval_s: float
    _records: list[dict[str, Any]] = field(default_factory=list)

    async def run(self) -> None:
        while True:
            t0 = time.time_ns()
            try:
                state = await self.motion_group_api.get_current_motion_group_state(
                    cell=self.cell, controller=self.controller, motion_group=self.motion_group
                )
                self._records.append(
                    {
                        "t_ns": t0,
                        "latency_ms": (time.time_ns() - t0) / 1e6,
                        "state": state.model_dump(mode="json", exclude_none=True),
                    }
                )
            except asyncio.CancelledError:
                raise
            except Exception as e:
                self._records.append(
                    {
                        "t_ns": t0,
                        "latency_ms": (time.time_ns() - t0) / 1e6,
                        "error": f"{type(e).__name__}: {e}",
                    }
                )
            await asyncio.sleep(self.interval_s)

    def write(self, path: Path) -> dict[str, Any]:
        with path.open("w") as f:
            for record in self._records:
                f.write(json.dumps(record, default=str) + "\n")
        return {"polls": len(self._records)}
