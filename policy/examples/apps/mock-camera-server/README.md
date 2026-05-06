# Mock Camera Server

WebRTC camera server that loops video from a HuggingFace dataset. Used for local development of vision-based robot policies without real cameras.

Streams 3 cameras (flange, left, right) at 480×640 RGB, 15fps over WebRTC.

## Run

```bash
cd policy/examples/apps/mock-camera-server
uv run python -m mock_camera_server
```

Open **http://localhost:9100** to see the camera feeds in the browser UI.

## Cameras

| Name | Device ID | Description |
|------|-----------|-------------|
| `flange` | `315122271048` | Wrist-mounted camera |
| `left` | `314522065367` | Left workspace camera |
| `right` | `319522063360` | Right workspace camera |

## API

| Method | Path | Description |
|--------|------|-------------|
| GET | `/` | Browser UI showing camera feeds |
| GET | `/status` | Active sessions and camera list |
| GET | `/api/devices/` | List available cameras |
| GET | `/api/devices/{id}/sensors/` | Sensor info for a device |
| POST | `/api/devices/{id}/stream/start` | Start camera stream |
| POST | `/api/devices/{id}/stream/stop` | Stop camera stream |
| POST | `/api/webrtc/offer` | Get WebRTC offer (SDP) |
| POST | `/api/webrtc/answer` | Send WebRTC answer (SDP) |

## Use with PolicyExecutor

```python
from policy import CameraConfig, CameraSet, PolicyExecutor

cameras = CameraSet(configs={
    "flange": CameraConfig(api_url="http://localhost:9100", device_id="315122271048"),
    "left": CameraConfig(api_url="http://localhost:9100", device_id="314522065367"),
    "right": CameraConfig(api_url="http://localhost:9100", device_id="319522063360"),
})

executor = PolicyExecutor(
    feature_map=feature_map,
    cameras=cameras,
    policy=my_policy,
    timeout_s=10.0,
)
result = await executor.run()

# Images arrive as numpy arrays in the observation:
# obs["flange"] -> np.ndarray shape=(480, 640, 3) dtype=uint8 (RGB)
```

## First run

On first run, videos are downloaded from HuggingFace. To pre-download:

```bash
cd policy/examples/apps/mock-camera-server
uv run python -c "
from huggingface_hub import hf_hub_download
import shutil, os
os.makedirs('videos', exist_ok=True)
for cam in ('flange', 'left', 'right'):
    path = hf_hub_download('StefanWagnerWandelbots/pusht_physical_15fps',
        f'videos/observation.images.{cam}/chunk-000/file-000.mp4', repo_type='dataset')
    shutil.copy(path, f'videos/{cam}.mp4')
"
```

Videos are from [`StefanWagnerWandelbots/pusht_physical_15fps`](https://huggingface.co/datasets/StefanWagnerWandelbots/pusht_physical_15fps) (480×640 RGB, 15fps).
