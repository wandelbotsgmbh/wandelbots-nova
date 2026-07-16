"""Lifecycle tests for the policy Rerun logger."""

from unittest.mock import AsyncMock, MagicMock

import pytest

from novapolicy.rerun.logger import PolicyRerunLogger


@pytest.mark.asyncio
async def test_stop_streaming_disconnects_the_dedicated_recording() -> None:
    policy_logger = PolicyRerunLogger([])
    streamer = MagicMock(stop=AsyncMock())
    recording = MagicMock()
    policy_logger.__dict__["_streamer"] = streamer
    policy_logger.__dict__["_recording"] = recording
    policy_logger.__dict__["_initialized"] = True

    await policy_logger.stop_streaming()

    streamer.stop.assert_awaited_once()
    recording.disconnect.assert_called_once_with()
    assert policy_logger.__dict__["_streamer"] is None
    assert policy_logger.__dict__["_recording"] is None
    assert policy_logger.__dict__["_initialized"] is False
