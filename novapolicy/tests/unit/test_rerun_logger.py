"""Lifecycle tests for the policy Rerun logger."""

import asyncio
import logging
import threading
from unittest.mock import AsyncMock, MagicMock

import pytest

from novapolicy.rerun import _is_rerun_active
from novapolicy.rerun.logger import PolicyRerunLogger

rr = pytest.importorskip("rerun")


def test_rerun_is_active_only_after_viewer_configuration(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    viewer = MagicMock(is_configured=False)
    manager = MagicMock()
    manager.get_viewer.return_value = viewer
    monkeypatch.setattr("nova.viewers.get_viewer_manager", lambda: manager)

    assert not _is_rerun_active()

    viewer.is_configured = True
    assert _is_rerun_active()


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


@pytest.mark.asyncio
async def test_stop_streaming_does_not_wait_for_a_blocked_rerun_disconnect(
    monkeypatch: pytest.MonkeyPatch,
    caplog: pytest.LogCaptureFixture,
) -> None:
    release_disconnect = threading.Event()
    recording = MagicMock()
    recording.disconnect.side_effect = release_disconnect.wait
    monkeypatch.setattr(rr, "RecordingStream", MagicMock(return_value=recording))
    monkeypatch.setattr(rr, "log", MagicMock())
    monkeypatch.setattr("novapolicy.rerun.blueprint.send_blueprint", MagicMock())
    monkeypatch.setattr("novapolicy.rerun.logger._RERUN_DISCONNECT_TIMEOUT_S", 0.01)

    policy_logger = PolicyRerunLogger([])
    await policy_logger.initialize()
    try:
        with caplog.at_level(logging.WARNING):
            await asyncio.wait_for(policy_logger.stop_streaming(), timeout=0.5)
    finally:
        release_disconnect.set()

    assert "Timed out disconnecting policy Rerun recording" in caplog.text
