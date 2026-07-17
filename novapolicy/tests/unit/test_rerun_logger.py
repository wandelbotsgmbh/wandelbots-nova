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
    streamer_factory = MagicMock(return_value=streamer)
    monkeypatch.setattr(rr, "RecordingStream", MagicMock(return_value=recording))
    monkeypatch.setattr(rr, "log", MagicMock())
    monkeypatch.setattr("novapolicy.rerun.blueprint.send_blueprint", MagicMock())
    monkeypatch.setattr(
        "novapolicy.rerun.streaming.StateStreamer",
        streamer_factory,
    )

    policy_logger = PolicyRerunLogger([], state_sample_interval_ms=10.0)
    await policy_logger.initialize()
    policy_logger.start_streaming({})
    await policy_logger.stop_streaming()
    await policy_logger.stop_streaming()

    assert streamer_factory.call_args.kwargs["state_sample_interval_ms"] == 10.0
    streamer.start.assert_called_once_with({})
    streamer.stop.assert_awaited_once()
    recording.disconnect.assert_called_once_with()
