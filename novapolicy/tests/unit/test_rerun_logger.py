"""Lifecycle tests for the policy Rerun logger."""

from unittest.mock import AsyncMock, MagicMock

import pytest

from novapolicy.rerun.logger import PolicyRerunLogger

rr = pytest.importorskip("rerun")


@pytest.mark.asyncio
async def test_stop_streaming_disconnects_the_dedicated_recording(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    recording = MagicMock()
    streamer = MagicMock(start=MagicMock(), stop=AsyncMock())
    monkeypatch.setattr(rr, "RecordingStream", MagicMock(return_value=recording))
    monkeypatch.setattr(rr, "log", MagicMock())
    monkeypatch.setattr("novapolicy.rerun.blueprint.send_blueprint", MagicMock())
    monkeypatch.setattr(
        "novapolicy.rerun.streaming.StateStreamer",
        MagicMock(return_value=streamer),
    )

    policy_logger = PolicyRerunLogger([])
    await policy_logger.initialize()
    policy_logger.start_streaming({})
    await policy_logger.stop_streaming()
    await policy_logger.stop_streaming()

    streamer.start.assert_called_once_with({})
    streamer.stop.assert_awaited_once()
    recording.disconnect.assert_called_once_with()
