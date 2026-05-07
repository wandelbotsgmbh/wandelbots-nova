# Mock Camera Server

WebRTC camera server that loops video from a HuggingFace dataset. For development without real cameras.

## Run

```bash
cd policy/examples/apps/mock-camera-server
uv run python -m mock_camera_server
```

Opens on **http://localhost:9100**. Streams 3 cameras (flange, left, right) at 480×640 RGB, 15fps.

## Use with PolicyExecutor

```python
from policy import CameraSet, WebRTCCameraConfig

cameras = CameraSet(configs={
    "flange": WebRTCCameraConfig(api_url="http://localhost:9100", device_id="315122271048"),
    "left": WebRTCCameraConfig(api_url="http://localhost:9100", device_id="314522065367"),
    "right": WebRTCCameraConfig(api_url="http://localhost:9100", device_id="319522063360"),
})
```

Images arrive as `numpy.ndarray` shape `(480, 640, 3)` dtype `uint8` (RGB).

## First run

Videos are downloaded from HuggingFace on first launch (~50MB).
