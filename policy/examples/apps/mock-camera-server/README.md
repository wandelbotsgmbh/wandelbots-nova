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
from policy import Observation, PolicySchema, WebRTCCameras

cameras = WebRTCCameras(api_url="http://localhost:9100", width=640, height=480, fps=15)

schema = PolicySchema(observations=[
    # ... joint observations ...
    Observation.image("flange", source=cameras.device("315122271048")),
    Observation.image("left", source=cameras.device("314522065367")),
    Observation.image("right", source=cameras.device("319522063360")),
])
```

Images arrive as `numpy.ndarray` shape `(480, 640, 3)` dtype `uint8` (RGB).

## First run

Videos are downloaded from HuggingFace on first launch (~50MB).
