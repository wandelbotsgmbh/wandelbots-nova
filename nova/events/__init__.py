from abc import ABC
from datetime import datetime, timedelta
from typing import Any, Literal
from uuid import UUID, uuid4

from blinker import signal
from pydantic import BaseModel, Field

from nova.cell.cell import Cell
from nova.cell.robot_cell import Device, OutputDevice
from nova.logging import logger
from nova.nats import Message as NatsMessage
from nova.nats import NatsClient

# TODO: when connect is called on multiple nova instances, every cycle.start triggers multiple cycle events
# TODO: check with Dirk
cycle_started = signal("cycle_started")
cycle_finished = signal("cycle_finished")
cycle_failed = signal("cycle_failed")

_NATS_CYCLE_SUBJECT = "nova.v2.cells.{cell}.cycle"


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


class CycleDevice(OutputDevice, Device):
    def __init__(self, cell: Cell):
        super().__init__()
        self._cycle = Cycle(cell=cell)

    async def write(self, key, _):
        if hasattr(self._cycle, key):
            method = getattr(self._cycle, key)
            await method()


class Cycle:
    """
    Context manager for tracking a process cycle in a robotic cell.

    The Cycle class provides a standardized way to track automation cycles,
    measure their execution time, and emit events for observability. It's designed
    to be used as an async context manager for automatic event handling.

    Events are emitted when:
    - A cycle starts (CycleStartedEvent)
    - A cycle finishes successfully (CycleFinishedEvent)
    - A cycle fails with an error (CycleFailedEvent)

    Example usage:
        ```python
        async with Cycle(cell) as cycle:
            # Your automation logic here
            await perform_task()
            # On successful completion, finish() is called automatically
        # If an exception occurs, fail() is called automatically
        ```

    Alternative manual usage:
        ```python
        cycle = Cycle(cell)
        try:
            await cycle.start()
            # Your automation logic here
            await perform_task()
            duration = await cycle.finish()
        except Exception as e:
            await cycle.fail(e)
        ```

    Attributes:
        cycle_id (UUID | None): Unique identifier for the cycle, set after start()
    """

    def __init__(self, cell: Cell):
        self.cycle_id: UUID | None = None
        self._timer = Timer()
        self._cell_id = cell.cell_id
        self._cycle_subject = _NATS_CYCLE_SUBJECT.format(cell=self._cell_id)

    async def start(self) -> datetime:
        """
        Start a new automation cycle and emit a CycleStartedEvent.

        This method starts the internal timer, generates a unique cycle ID,
        and sends a notification that a new cycle has begun.

        Returns:
            datetime: The timestamp when the cycle started

        Raises:
            RuntimeError: If the cycle has already been started
        """
        try:
            start_time = self._timer.start()
        except RuntimeError as e:
            raise RuntimeError("Cycle already started") from e

        self.cycle_id = uuid4()
        event = CycleStartedEvent(cycle_id=self.cycle_id, timestamp=start_time, cell=self._cell_id)
        logger.debug(f"Cycle started with ID: {self.cycle_id}")

        # Emit blinker signal if available
        if cycle_started is not None:
            logger.debug(
                f"Emitting cycle_started signal with {len(cycle_started.receivers)} connected receivers"
            )
            cycle_started.send(self, message=event)

        return start_time

    async def finish(self) -> timedelta:
        """
        Mark the automation cycle as successfully completed and emit a CycleFinishedEvent.

        This method stops the internal timer, calculates the cycle duration,
        and sends a notification that the cycle has completed successfully.

        Returns:
            timedelta: The total duration of the cycle

        Raises:
            RuntimeError: If the cycle has not been started
            AssertionError: If cycle_id is None (start() was never called)
        """
        try:
            end_time = self._timer.stop()
        except RuntimeError as e:
            raise RuntimeError("Cycle not started") from e

        assert self.cycle_id is not None, "Cycle ID is missing; ensure start() was called first"

        duration_ms = int((end_time - self._timer.start_time).total_seconds() * 1000)
        event = CycleFinishedEvent(
            cycle_id=self.cycle_id, timestamp=end_time, duration_ms=duration_ms, cell=self._cell_id
        )

        logger.debug(f"Cycle finished with ID: {self.cycle_id}")

        # Emit blinker signal if available
        if cycle_finished is not None:
            logger.debug(
                f"Emitting cycle_finished signal with {len(cycle_finished.receivers)} connected receivers"
            )
            cycle_finished.send(self, message=event)

        cycle_time = self._timer.elapsed()
        self._timer.reset()
        return cycle_time

    async def fail(self, reason: Exception | str) -> None:
        """
        Mark the automation cycle as failed and emit a CycleFailedEvent.

        This method stops the internal timer and sends a notification
        that the cycle has failed with the provided reason.

        Args:
            reason: The reason for failure, either as a string or an Exception

        Raises:
            ValueError: If an empty reason is provided
            RuntimeError: If the cycle has not been started
            AssertionError: If cycle_id is None (start() was never called)
        """
        if not reason:
            raise ValueError("Reason for failure must be provided")

        try:
            failure_time = self._timer.stop()
        except RuntimeError as e:
            raise RuntimeError("Cycle not started") from e

        assert self.cycle_id is not None, "Cycle ID is missing; ensure start() was called first"

        if isinstance(reason, Exception):
            reason = str(reason)
        event = CycleFailedEvent(
            cycle_id=self.cycle_id, timestamp=failure_time, cell=self._cell_id, reason=reason
        )
        logger.info(f"Cycle failed with ID: {self.cycle_id}, reason: {reason}")

        # Emit blinker signal if available
        if cycle_failed is not None:
            cycle_failed.send(self, message=event)

        self._timer.reset()

    async def __aenter__(self):
        """
        Async context manager entry point that starts the cycle.

        Returns:
            Cycle: The cycle instance for use within the context
        """
        await self.start()
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        """
        Async context manager exit point that completes the cycle.

        Automatically calls finish() on successful completion or
        fail() if an exception occurred within the context.

        Args:
            exc_type: The exception type if an exception was raised, otherwise None
            exc_val: The exception value if an exception was raised, otherwise None
            exc_tb: The traceback if an exception was raised, otherwise None

        Returns:
            bool: True to suppress exceptions (prevents exceptions from propagating)
        """
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
    reason: str = Field(..., description="Human-readable explanation of failure")


async def cycle_event_handler(
    sender: Any, message: BaseCycleEvent, nats_client: NatsClient
) -> None:
    """
    Event handler that publishes cycle events to NATS using the nats client.

    Args:
        sender: The signal sender (blinker signal)
        message: The cycle event message (BaseCycleEvent)
        nats_client: The NATS client for publishing events
        **kwargs: Additional keyword arguments
    """
    logger.debug(f"NATS event handler called with message: {message}, sender: {sender}")

    event_type = message.event_type
    logger.debug(f"NATS event handler received {event_type} event")

    try:
        # Use the provided nats client

        subject = f"nova.v2.cells.{message.cell}.cycle"
        nats_message = NatsMessage(subject=subject, data=message.model_dump_json().encode())

        await nats_client.publish_message(nats_message)
        logger.debug(f"Published {event_type} event to NATS subject: {subject}")

    except Exception as e:
        logger.error(f"Failed to publish {event_type} event to NATS: {e}")


# Export key classes and signals for easy import
__all__ = [
    "Timer",
    "CycleDevice",
    "Cycle",
    "BaseCycleEvent",
    "CycleStartedEvent",
    "CycleFinishedEvent",
    "CycleFailedEvent",
    "cycle_started",
    "cycle_finished",
    "cycle_failed",
    "cycle_event_handler",
]
