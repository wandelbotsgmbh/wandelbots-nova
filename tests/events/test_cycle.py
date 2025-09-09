import json
from datetime import datetime, timedelta
from unittest.mock import AsyncMock, MagicMock, patch
from uuid import UUID

import pytest

from nova import Cell
from nova.events import Cycle, CycleFailedEvent, CycleFinishedEvent, CycleStartedEvent


@pytest.fixture
def mock_cell():
    cell = MagicMock(spec=Cell)
    cell.cell_id = "test-cell-1"
    cell.nats = AsyncMock()
    return cell


class TestCycle:
    @pytest.mark.asyncio
    async def test_init(self, mock_cell):
        """Test initialization of the Cycle class."""
        cycle = Cycle(mock_cell)
        assert cycle.cycle_id is None
        assert cycle._cell_id == "test-cell-1"

    @pytest.mark.asyncio
    async def test_start(self, mock_cell):
        """Test the start method."""
        cycle = Cycle(mock_cell)

        # Mock NATS publish_message to avoid actual event dispatch
        with patch.object(mock_cell.nats, "publish_message", new=AsyncMock()) as mock_publish:
            start_time = await cycle.start()

            # Verify return value and state changes
            assert isinstance(start_time, datetime)
            assert cycle.cycle_id is not None
            assert isinstance(cycle.cycle_id, UUID)

            # Verify NATS publish was called correctly
            mock_publish.assert_called_once()
            published_message = mock_publish.call_args[0][0]
            assert published_message.subject == "nova.v2.cells.test-cell-1.cycle"

            # Parse and verify the event data
            event_data = json.loads(published_message.data.decode())
            event = CycleStartedEvent.model_validate(event_data)
            assert event.cycle_id == cycle.cycle_id
            assert event.cell == "test-cell-1"
            assert event.timestamp == start_time

    @pytest.mark.asyncio
    async def test_start_already_running(self, mock_cell):
        """Test that starting an already running cycle raises an error."""
        cycle = Cycle(mock_cell)

        with patch.object(mock_cell.nats, "publish_message", new=AsyncMock()):
            await cycle.start()

            with pytest.raises(RuntimeError, match="Cycle already started"):
                await cycle.start()

    @pytest.mark.asyncio
    async def test_finish(self, mock_cell):
        """Test the finish method."""
        cycle = Cycle(mock_cell)

        # Start the cycle first
        with patch.object(mock_cell.nats, "publish_message", new=AsyncMock()):
            await cycle.start()

        # Now test finish
        with patch.object(mock_cell.nats, "publish_message", new=AsyncMock()) as mock_publish:
            cycle_time = await cycle.finish()

            # Verify return value and state changes
            assert isinstance(cycle_time, timedelta)

            # Verify NATS publish was called correctly
            mock_publish.assert_called_once()
            published_message = mock_publish.call_args[0][0]
            assert published_message.subject == "nova.v2.cells.test-cell-1.cycle"

            # Parse and verify the event data
            event_data = json.loads(published_message.data.decode())
            event = CycleFinishedEvent.model_validate(event_data)
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
        with patch.object(mock_cell.nats, "publish_message", new=AsyncMock()):
            await cycle.start()

        # Now test fail
        with patch.object(mock_cell.nats, "publish_message", new=AsyncMock()) as mock_publish:
            await cycle.fail("Something went wrong")

            # Verify NATS publish was called correctly
            mock_publish.assert_called_once()
            published_message = mock_publish.call_args[0][0]
            assert published_message.subject == "nova.v2.cells.test-cell-1.cycle"

            # Parse and verify the event data
            event_data = json.loads(published_message.data.decode())
            event = CycleFailedEvent.model_validate(event_data)
            assert event.cycle_id == cycle.cycle_id
            assert event.cell == "test-cell-1"
            assert isinstance(event.timestamp, datetime)
            assert event.reason == "Something went wrong"

    @pytest.mark.asyncio
    async def test_fail_with_exception(self, mock_cell):
        """Test the fail method with an exception reason."""
        cycle = Cycle(mock_cell)

        # Start the cycle first
        with patch.object(mock_cell.nats, "publish_message", new=AsyncMock()):
            await cycle.start()

        # Now test fail with exception
        exception = ValueError("Invalid value")
        with patch.object(mock_cell.nats, "publish_message", new=AsyncMock()) as mock_publish:
            await cycle.fail(exception)

            # Verify the published message
            published_message = mock_publish.call_args[0][0]

            # Parse and verify the event data
            event_data = json.loads(published_message.data.decode())
            event = CycleFailedEvent.model_validate(event_data)

            # Verify event reason contains exception message
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
        with patch.object(mock_cell.nats, "publish_message", new=AsyncMock()):
            await cycle.start()

        with pytest.raises(ValueError, match="Reason for failure must be provided"):
            await cycle.fail("")

    @pytest.mark.asyncio
    async def test_context_manager_success(self, mock_cell):
        """Test the context manager with successful execution."""
        with patch.object(mock_cell.nats, "publish_message", new=AsyncMock()) as mock_publish:
            async with Cycle(mock_cell) as cycle:
                assert cycle.cycle_id is not None
                assert isinstance(cycle.cycle_id, UUID)

            # Verify both start and finish events were published
            assert mock_publish.call_count == 2

            # Verify both events were published to the correct subject
            all_calls = mock_publish.call_args_list
            start_message = all_calls[0][0][0]
            finish_message = all_calls[1][0][0]
            assert start_message.subject == "nova.v2.cells.test-cell-1.cycle"
            assert finish_message.subject == "nova.v2.cells.test-cell-1.cycle"

    @pytest.mark.asyncio
    async def test_context_manager_exception(self, mock_cell):
        """Test the context manager with an exception."""
        with patch.object(mock_cell.nats, "publish_message", new=AsyncMock()) as mock_publish:
            try:
                async with Cycle(mock_cell):
                    raise ValueError("Test exception")
            except ValueError:
                pass  # Exception should be suppressed by context manager

            # Verify both start and fail events were published
            assert mock_publish.call_count == 2

            # Check first event (start)
            start_message = mock_publish.call_args_list[0][0][0]
            assert start_message.subject == "nova.v2.cells.test-cell-1.cycle"
            start_event_data = json.loads(start_message.data.decode())
            start_event = CycleStartedEvent.model_validate(start_event_data)
            assert start_event.cell == "test-cell-1"

            # Check second event (fail)
            fail_message = mock_publish.call_args_list[1][0][0]
            assert fail_message.subject == "nova.v2.cells.test-cell-1.cycle"
            fail_event_data = json.loads(fail_message.data.decode())
            fail_event = CycleFailedEvent.model_validate(fail_event_data)
            assert "Test exception" in fail_event.reason

    @pytest.mark.asyncio
    async def test_cycle_with_extra(self, mock_cell):
        # TODO: mock environment variable
        with patch.object(mock_cell.nats, "publish_message", new=AsyncMock()) as mock_publish:
            async with Cycle(mock_cell, extra={"key1": "value1", "key2": "value2"}):
                pass

            # Verify both start and finish events were published
            assert mock_publish.call_count == 2

            # Check start event
            start_message = mock_publish.call_args_list[0][0][0]
            start_event_data = json.loads(start_message.data.decode())
            start_event = CycleStartedEvent.model_validate(start_event_data)

            assert start_event.extra["key1"] == "value1"
            assert start_event.extra["key2"] == "value2"

            # Check finish event
            finish_message = mock_publish.call_args_list[1][0][0]
            finish_event_data = json.loads(finish_message.data.decode())
            finish_event = CycleFinishedEvent.model_validate(finish_event_data)
            assert finish_event.extra["key1"] == "value1"
            assert finish_event.extra["key2"] == "value2"

        with patch.object(mock_cell.nats, "publish_message", new=AsyncMock()) as mock_publish:
            try:
                async with Cycle(mock_cell, extra={"key1": "value1", "key2": "value2"}):
                    raise ValueError("Test exception")
            except ValueError:
                pass

            # Verify both start and fail events were published
            assert mock_publish.call_count == 2

            # Check start event
            start_message = mock_publish.call_args_list[0][0][0]
            start_event_data = json.loads(start_message.data.decode())
            start_event = CycleStartedEvent.model_validate(start_event_data)
            assert finish_event.extra["key1"] == "value1"
            assert finish_event.extra["key2"] == "value2"

            # Check fail event
            fail_message = mock_publish.call_args_list[1][0][0]
            fail_event_data = json.loads(fail_message.data.decode())
            fail_event = CycleFailedEvent.model_validate(fail_event_data)
            assert fail_event.extra["key1"] == "value1"
            assert fail_event.extra["key2"] == "value2"
