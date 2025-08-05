from datetime import datetime, timedelta
from unittest.mock import AsyncMock, patch
from uuid import UUID

import pytest

from nova.events import (
    Cycle,
    CycleFailedEvent,
    CycleFinishedEvent,
    CycleStartedEvent,
    cycle_failed,
    cycle_finished,
    cycle_started,
)


@pytest.fixture
def mock_cell():
    return "test-cell-1"


class TestCycle:
    @pytest.mark.asyncio
    async def test_init(self, mock_cell):
        """Test initialization of the Cycle class."""
        cycle = Cycle(mock_cell)
        assert cycle.cycle_id is None
        assert cycle._cell_id == mock_cell

    @pytest.mark.asyncio
    async def test_start(self, mock_cell):
        """Test the start method."""
        cycle = Cycle(mock_cell)

        # Mock the signal.send_async to avoid actual event dispatch
        with patch.object(cycle_started, "send_async", new=AsyncMock()) as mock_send:
            start_time = await cycle.start()

            # Verify return value and state changes
            assert isinstance(start_time, datetime)
            assert cycle.cycle_id is not None
            assert isinstance(cycle.cycle_id, UUID)

            # Verify event was sent correctly
            mock_send.assert_called_once()
            assert mock_send.call_args[0][0] == cycle
            event = mock_send.call_args[1]["message"]
            assert isinstance(event, CycleStartedEvent)
            assert event.cycle_id == cycle.cycle_id
            assert event.cell == "test-cell-1"
            assert event.timestamp == start_time

    @pytest.mark.asyncio
    async def test_start_already_running(self, mock_cell):
        """Test that starting an already running cycle raises an error."""
        cycle = Cycle(mock_cell)

        with patch.object(cycle_started, "send_async", new=AsyncMock()):
            await cycle.start()

            with pytest.raises(RuntimeError, match="Cycle already started"):
                await cycle.start()

    @pytest.mark.asyncio
    async def test_finish(self, mock_cell):
        """Test the finish method."""
        cycle = Cycle(mock_cell)

        # Start the cycle first
        with patch.object(cycle_started, "send_async", new=AsyncMock()):
            await cycle.start()

        # Now test finish
        with patch.object(cycle_finished, "send_async", new=AsyncMock()) as mock_send:
            cycle_time = await cycle.finish()

            # Verify return value and state changes
            assert isinstance(cycle_time, timedelta)

            # Verify event was sent correctly
            mock_send.assert_called_once()
            assert mock_send.call_args[0][0] == cycle
            event = mock_send.call_args[1]["message"]
            assert isinstance(event, CycleFinishedEvent)
            assert event.cycle_id == cycle.cycle_id
            assert event.cell == "test-cell-1"
            assert isinstance(event.timestamp, datetime)
            assert isinstance(event.duration_ms, int)
            assert event.duration_ms >= 0  # Should be non-negative

    @pytest.mark.asyncio
    async def test_finish_not_started(self, mock_cell):
        """Test that finishing a non-started cycle raises an error."""
        cycle = Cycle(mock_cell)

        with pytest.raises(RuntimeError, match="Cycle not started"):
            await cycle.finish()

    @pytest.mark.asyncio
    async def test_fail(self, mock_cell):
        """Test the fail method with a string reason."""
        cycle = Cycle(mock_cell)

        # Start the cycle first
        with patch.object(cycle_started, "send_async", new=AsyncMock()):
            await cycle.start()

        # Now test fail
        with patch.object(cycle_failed, "send_async", new=AsyncMock()) as mock_send:
            await cycle.fail("Something went wrong")

            # Verify event was sent correctly
            mock_send.assert_called_once()
            assert mock_send.call_args[0][0] == cycle
            event = mock_send.call_args[1]["message"]
            assert isinstance(event, CycleFailedEvent)
            assert event.cycle_id == cycle.cycle_id
            assert event.cell == "test-cell-1"
            assert isinstance(event.timestamp, datetime)
            assert event.reason == "Something went wrong"

    @pytest.mark.asyncio
    async def test_fail_with_exception(self, mock_cell):
        """Test the fail method with an exception reason."""
        cycle = Cycle(mock_cell)

        # Start the cycle first
        with patch.object(cycle_started, "send_async", new=AsyncMock()):
            await cycle.start()

        # Now test fail with exception
        exception = ValueError("Invalid value")
        with patch.object(cycle_failed, "send_async", new=AsyncMock()) as mock_send:
            await cycle.fail(exception)

            # Verify event reason contains exception message
            event = mock_send.call_args[1]["message"]
            assert "Invalid value" in event.reason

    @pytest.mark.asyncio
    async def test_fail_not_started(self, mock_cell):
        """Test that failing a non-started cycle raises an error."""
        cycle = Cycle(mock_cell)

        with pytest.raises(RuntimeError, match="Cycle not started"):
            await cycle.fail("Error")

    @pytest.mark.asyncio
    async def test_fail_empty_reason(self, mock_cell):
        """Test that failing with an empty reason raises an error."""
        cycle = Cycle(mock_cell)

        with patch.object(cycle_started, "send_async", new=AsyncMock()):
            await cycle.start()

        with pytest.raises(ValueError, match="Reason for failure must be provided"):
            await cycle.fail("")

    @pytest.mark.asyncio
    async def test_context_manager_success(self, mock_cell):
        """Test the context manager with successful execution."""
        with (
            patch.object(cycle_started, "send_async", new=AsyncMock()) as mock_start,
            patch.object(cycle_finished, "send_async", new=AsyncMock()) as mock_finish,
            patch.object(cycle_failed, "send_async", new=AsyncMock()) as mock_fail,
        ):
            async with Cycle(mock_cell) as cycle:
                assert cycle.cycle_id is not None
                assert isinstance(cycle.cycle_id, UUID)

            # Verify start and finish were called, but not fail
            mock_start.assert_called_once()
            mock_finish.assert_called_once()
            mock_fail.assert_not_called()

    @pytest.mark.asyncio
    async def test_context_manager_exception(self, mock_cell):
        """Test the context manager with an exception."""
        with (
            patch.object(cycle_started, "send_async", new=AsyncMock()) as mock_start,
            patch.object(cycle_finished, "send_async", new=AsyncMock()) as mock_finish,
            patch.object(cycle_failed, "send_async", new=AsyncMock()) as mock_fail,
        ):
            try:
                async with Cycle(mock_cell):
                    raise ValueError("Test exception")
            except ValueError:
                pass  # Exception should be suppressed by context manager

            # Verify start and fail were called, but not finish
            mock_start.assert_called_once()
            mock_finish.assert_not_called()
            mock_fail.assert_called_once()
            event = mock_fail.call_args[1]["message"]
            assert "Test exception" in event.reason
