from datetime import datetime, timedelta
from unittest.mock import AsyncMock, MagicMock, patch
from uuid import UUID

import pytest

from nova import Cell
from nova.events import Cycle, CycleFailedEvent, CycleFinishedEvent, CycleStartedEvent
from nova.nats import Message


@pytest.fixture
def mock_cell():
    cell = MagicMock(spec=Cell)
    cell._api_gateway = MagicMock()
    cell.cell_id = "test-cell-1"
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

        # Mock the signal.send_async to avoid actual event dispatch
        with patch.object(
            mock_cell._api_gateway, "publish_message", new=AsyncMock()
        ) as mock_publish_message:
            start_time = await cycle.start()

            # Verify return value and state changes
            assert isinstance(start_time, datetime)
            assert cycle.cycle_id is not None
            assert isinstance(cycle.cycle_id, UUID)

            # Verify event was sent correctly
            mock_publish_message.assert_called_once()
            nats_message: Message = mock_publish_message.call_args[0][0]
            assert isinstance(nats_message, Message)
            assert nats_message.subject == "nova.cells.test-cell-1.cycle"
            cycle_started = CycleStartedEvent.model_validate_json(nats_message.data)
            assert cycle_started.cell == "test-cell-1"
            assert cycle_started.cycle_id == cycle.cycle_id
            assert cycle_started.timestamp == start_time

    @pytest.mark.asyncio
    async def test_start_already_running(self, mock_cell):
        """Test that starting an already running cycle raises an error."""
        cycle = Cycle(mock_cell)

        with patch.object(mock_cell._api_gateway, "publish_message", new=AsyncMock()):
            await cycle.start()

            with pytest.raises(RuntimeError, match="Cycle already started"):
                await cycle.start()

    @pytest.mark.asyncio
    async def test_finish(self, mock_cell):
        """Test the finish method."""
        cycle = Cycle(mock_cell)

        # Start the cycle first
        with patch.object(mock_cell._api_gateway, "publish_message", new=AsyncMock()):
            await cycle.start()

        # Now test finish
        with patch.object(
            mock_cell._api_gateway, "publish_message", new=AsyncMock()
        ) as mock_publish_message:
            cycle_time = await cycle.finish()

            # Verify return value and state changes
            assert isinstance(cycle_time, timedelta)

            # Verify event was sent correctly
            mock_publish_message.assert_called_once()
            nats_message: Message = mock_publish_message.call_args[0][0]
            assert isinstance(nats_message, Message)
            assert nats_message.subject == "nova.cells.test-cell-1.cycle"
            cycle_finished = CycleFinishedEvent.model_validate_json(nats_message.data)
            assert cycle_finished.cell == "test-cell-1"
            assert cycle_finished.cycle_id == cycle.cycle_id
            assert isinstance(cycle_finished.timestamp, datetime)
            assert isinstance(cycle_finished.duration_ms, int)
            assert cycle_finished.duration_ms >= 0  # Should be non-negative

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
        with patch.object(mock_cell._api_gateway, "publish_message", new=AsyncMock()):
            await cycle.start()

        # Now test fail
        with patch.object(
            mock_cell._api_gateway, "publish_message", new=AsyncMock()
        ) as mock_publish_message:
            await cycle.fail("Something went wrong")

            # Verify event was sent correctly
            mock_publish_message.assert_called_once()
            nats_message: Message = mock_publish_message.call_args[0][0]
            assert isinstance(nats_message, Message)
            assert nats_message.subject == "nova.cells.test-cell-1.cycle"
            cycle_failed = CycleFailedEvent.model_validate_json(nats_message.data)
            assert cycle_failed.cycle_id == cycle.cycle_id
            assert cycle_failed.cell == "test-cell-1"
            assert isinstance(cycle_failed.timestamp, datetime)
            assert cycle_failed.reason == "Something went wrong"

    @pytest.mark.asyncio
    async def test_fail_with_exception(self, mock_cell):
        """Test the fail method with an exception reason."""
        cycle = Cycle(mock_cell)

        # Start the cycle first
        with patch.object(mock_cell._api_gateway, "publish_message", new=AsyncMock()):
            await cycle.start()

        # Now test fail with exception
        exception = ValueError("Invalid value")
        with patch.object(
            mock_cell._api_gateway, "publish_message", new=AsyncMock()
        ) as mock_publish_message:
            await cycle.fail(exception)

            # Verify event reason contains exception message
            mock_publish_message.assert_called_once()
            nats_message: Message = mock_publish_message.call_args[0][0]
            cycle_failed = CycleFailedEvent.model_validate_json(nats_message.data)
            assert "Invalid value" in cycle_failed.reason

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

        with patch.object(mock_cell._api_gateway, "publish_message", new=AsyncMock()):
            await cycle.start()

        with pytest.raises(ValueError, match="Reason for failure must be provided"):
            await cycle.fail("")

    @pytest.mark.asyncio
    async def test_context_manager_success(self, mock_cell):
        """Test the context manager with successful execution."""
        with patch.object(mock_cell._api_gateway, "publish_message", new=AsyncMock()):
            async with Cycle(mock_cell) as cycle:
                assert cycle.cycle_id is not None
                assert isinstance(cycle.cycle_id, UUID)

    @pytest.mark.asyncio
    async def test_context_manager_exception(self, mock_cell):
        """Test the context manager with an exception."""
        with patch.object(mock_cell._api_gateway, "publish_message", new=AsyncMock()):
            try:
                async with Cycle(mock_cell):
                    raise ValueError("Test exception")
            except ValueError:
                pass  # Exception should be suppressed by context manager
