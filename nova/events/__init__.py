from abc import ABC
import time
from datetime import datetime, timedelta
from typing import Optional, Literal
from uuid import UUID, uuid4

from blinker import signal
from pydantic import BaseModel, Field


cycle_started = signal("cycle_started")
cycle_finished = signal("cycle_finished")
cycle_failed = signal("cycle_failed")


class Timer:
    def __init__(self):
        self.start_time = None
        self.stop_time = None

    def start(self) -> datetime:
        if self.start_time is not None:
            raise RuntimeError("Timer is already running.")
        self.start_time = datetime.now()
        return self.start_time

    def stop(self) -> datetime:
        if self.start_time is None:
            raise RuntimeError("Timer is not running.")
        self.stop_time = datetime.now()
        return self.stop_time

    def reset(self) -> None:
        self.start_time = None
        self.stop_time = None

    def elapsed(self) -> timedelta:
        if self.stop_time is None:
            return datetime.now() - self.start_time
        return self.stop_time - self.start_time

    def is_running(self) -> bool:
        return self.start_time is not None and self.stop_time is None
    

class Cycle:
    def __init__(self, cell_id: str):
        self.cycle_id: UUID | None = None
        self._cell_id = cell_id
        self._timer = Timer()

    async def start(self):
        try:
            start_time = self._timer.start()
        except RuntimeError as e:
            raise RuntimeError("Cycle already started") from e
        
        self.cycle_id = uuid4()
        event = CycleStartedEvent(cycle_id=self.cycle_id, timestamp=start_time, cell=self._cell_id)
        await cycle_started.send_async("nova", message=event)

    async def finish(self):
        try:
            end_time = self._timer.stop()
        except RuntimeError as e:
            raise RuntimeError("Cycle not started") from e
        
        duration_ms = int((end_time - self._timer.start_time).total_seconds() * 1000)
        event = CycleFinishedEvent(cycle_id=self.cycle_id, timestamp=end_time, duration_ms=duration_ms, cell=self._cell_id)
        await cycle_finished.send_async("nova", message=event)
        self._timer.reset()

    async def fail(self, reason: str):
        try:
            failure_time = self._timer.stop()
        except RuntimeError as e:
            raise RuntimeError("Cycle not started") from e
        
        event = CycleFailedEvent(cycle_id=self.cycle_id, timestamp=failure_time, cell=self._cell_id, reason=reason)
        await cycle_failed.send_async("nova", message=event)
        self._timer.reset()

    async def __aenter__(self):
        await self.start()
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        if exc_type is None:
            await self.finish()
        else:
            await self.fail(str(exc_val))
        return True


class BaseCycleEvent(BaseModel, ABC):
    event_type: Literal["cycle_started", "cycle_finished", "cycle_failed"]
    id: UUID = Field(default_factory=uuid4, description="Unique event identifier")
    cycle_id: UUID = Field(..., description="Unique identifier for the automation cycle")
    timestamp: datetime = Field(..., description="Event creation time (ISO 8601, UTC)")
    cell: str = Field(..., description="Identifier of the robotic cell")


class CycleStartedEvent(BaseCycleEvent):
    event_type: Literal["cycle_started"] = "cycle_started"


class CycleFinishedEvent(BaseCycleEvent):
    event_type: Literal["cycle_finished"] = "cycle_finished"
    duration_ms: int = Field(..., description="Cycle duration in milliseconds")


class CycleFailedEvent(BaseCycleEvent):
    event_type: Literal["cycle_failed"] = "cycle_failed"
    reason: str = Field(None, description="Human-readable explanation of failure")
