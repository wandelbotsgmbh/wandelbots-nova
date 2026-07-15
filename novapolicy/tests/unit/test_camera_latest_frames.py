"""Latest camera frames used by continuous Rerun logging."""

import asyncio
from unittest.mock import MagicMock

import numpy as np
import pytest

from novapolicy.cameras.webrtc import WebRTCCameraConfig, WebRTCDevice
from novapolicy.rerun.streaming import StateStreamer
import rerun as rr


def test_webrtc_latest_frame_does_not_advance_policy_frame_history() -> None:
    device = WebRTCDevice(
        WebRTCCameraConfig(api_url="http://camera", device_id="scene"),
        frame_history=2,
    )
    connection = MagicMock()
    connection.frame_age_s.return_value = 0.0
    device.__dict__["_connection"] = connection

    first_frame = np.full((2, 2, 3), 1, dtype=np.uint8)
    latest_frame = np.full((2, 2, 3), 2, dtype=np.uint8)
    second_frame = np.full((2, 2, 3), 3, dtype=np.uint8)

    connection.latest_frame.return_value = first_frame
    first_observation = device.read()
    connection.latest_frame.return_value = latest_frame
    latest = device.get_latest_frame()
    connection.latest_frame.return_value = second_frame
    second_observation = device.read()

    assert np.array_equal(first_observation, np.stack([first_frame, first_frame]))
    assert np.array_equal(latest, latest_frame)
    assert np.array_equal(second_observation, np.stack([first_frame, second_frame]))


@pytest.mark.asyncio
async def test_state_streamer_logs_latest_camera_frames_between_policy_chunks(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    read_images = MagicMock(return_value={"scene": np.zeros((2, 2, 3), dtype=np.uint8)})
    log_images = MagicMock()
    streamer = StateStreamer(
        start_time=0.0,
        dh_robots={},
        visualizers={},
        tcp_trail={},
        max_trail_points=10,
        image_reader=read_images,
    )
    monkeypatch.setattr(streamer, "_log_images", log_images)
    monkeypatch.setattr(rr, "set_time", MagicMock())

    streamer.start({})
    await asyncio.sleep(0.18)
    await streamer.stop()

    assert read_images.call_count >= 2
    assert log_images.call_count == read_images.call_count
