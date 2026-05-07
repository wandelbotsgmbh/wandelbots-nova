"""Mock GR00T policy service for a dual-arm UR10e embodiment.

Implements the exact same ZMQ REQ/REP protocol as ``gr00t.policy.server_client.PolicyServer``:
- ``ping`` → ``{"status": "ok", "message": "Server is running"}``
- ``kill`` → stops the server
- ``get_action(observation, options)`` → ``(action_dict, info_dict)``
- ``reset(options)`` → ``{}``
- ``get_modality_config()`` → modality metadata (with ``__ModalityConfig_class__`` envelope)

Embodiment: dual-arm UR10e (6-DOF each) with grippers and cameras.

Expected observation:
- ``state.left_arm``: (B=1, T=1, 6) float32 — left arm joint positions
- ``state.right_arm``: (B=1, T=1, 6) float32 — right arm joint positions
- ``state.left_eef_9d``: (B=1, T=1, 9) float32 — left TCP (XYZ meters + rot6d)
- ``state.right_eef_9d``: (B=1, T=1, 9) float32 — right TCP
- ``state.left_gripper``: (B=1, T=1, 1) float32 — left gripper position
- ``state.right_gripper``: (B=1, T=1, 1) float32 — right gripper position
- ``video.flange``: (B=1, T=1, H, W, 3) uint8 — camera image (at least one required)
- ``language.annotation.language.language_instruction``: [["instruction"]]

Actions returned (absolute joint targets):
- ``left_arm``: (B=1, ACTION_HORIZON, 6) float32
- ``right_arm``: (B=1, ACTION_HORIZON, 6) float32
- ``left_gripper``: (B=1, ACTION_HORIZON, 1) float32
- ``right_gripper``: (B=1, ACTION_HORIZON, 1) float32
"""

from __future__ import annotations

import io
import logging
import math
import threading
import time
from collections.abc import Iterator
from contextlib import asynccontextmanager
from typing import Any

import msgpack
import numpy as np
import uvicorn
from decouple import config
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

BASE_PATH = config("BASE_PATH", default="", cast=str)
ZMQ_HOST = config("ZMQ_HOST", default="0.0.0.0", cast=str)  # noqa: S104
ZMQ_PORT = config("ZMQ_PORT", default=5555, cast=int)
API_TOKEN: str | None = config("GROOT_API_TOKEN", default="")
if not API_TOKEN:
    API_TOKEN = None

# Embodiment configuration for dual-arm UR10e
STATE_KEYS = ("left_arm", "right_arm", "left_eef_9d", "right_eef_9d", "left_gripper", "right_gripper")
ACTION_KEYS = ("left_arm", "right_arm", "left_gripper", "right_gripper")
VIDEO_KEYS = ("flange",)
LANGUAGE_KEY = "annotation.language.language_instruction"
VIDEO_DELTA_INDICES = [0]  # T=1 for finetuned model
STATE_DELTA_INDICES = [0]  # T=1

ACTION_HORIZON = 16
AMPLITUDE = 0.3  # radians


# ---------------------------------------------------------------------------
# Msgpack serializer (identical to gr00t.policy.server_client.MsgSerializer)
# ---------------------------------------------------------------------------


class MsgSerializer:
    @staticmethod
    def to_bytes(data: Any) -> bytes:
        return msgpack.packb(data, default=MsgSerializer._encode_custom_classes)

    @staticmethod
    def from_bytes(data: bytes) -> Any:
        return msgpack.unpackb(data, object_hook=MsgSerializer._decode_custom_classes)

    @staticmethod
    def _decode_custom_classes(obj: object) -> object:
        if not isinstance(obj, dict) or "__ndarray_class__" not in obj:
            return obj
        array_bytes = obj.get("as_npy")
        if not isinstance(array_bytes, bytes):
            msg = "Invalid ndarray payload"
            raise TypeError(msg)
        return np.load(io.BytesIO(array_bytes), allow_pickle=False)

    @staticmethod
    def _encode_custom_classes(obj: object) -> object:
        if not isinstance(obj, np.ndarray):
            return obj
        output = io.BytesIO()
        np.save(output, obj, allow_pickle=False)
        return {"__ndarray_class__": True, "as_npy": output.getvalue()}


# ---------------------------------------------------------------------------
# Observation validation (mirrors gr00t.policy.gr00t_policy.check_observation)
# ---------------------------------------------------------------------------


def _check_observation(observation: dict[str, Any]) -> None:
    """Validate observation structure and types like the real GR00T server."""
    for modality in ("video", "state", "language"):
        if modality not in observation:
            msg = f"Observation must contain a '{modality}' key"
            raise ValueError(msg)
        if not isinstance(observation[modality], dict):
            msg = f"Observation '{modality}' must be a dictionary"
            raise TypeError(msg)

    # Video validation
    video_obs = observation["video"]
    if not video_obs:
        msg = f"Observation must contain at least one video key. Expected: {list(VIDEO_KEYS)}"
        raise ValueError(msg)
    for video_key in video_obs:
        arr = video_obs[video_key]
        if not isinstance(arr, np.ndarray):
            msg = f"Video key '{video_key}' must be a numpy array"
            raise TypeError(msg)
        if arr.dtype != np.uint8:
            msg = f"Video key '{video_key}' must be uint8, got {arr.dtype}"
            raise TypeError(msg)
        if arr.ndim != 5:
            msg = f"Video key '{video_key}' must be (B, T, H, W, C), got shape {arr.shape}"
            raise ValueError(msg)
        expected_t = len(VIDEO_DELTA_INDICES)
        if arr.shape[1] != expected_t:
            msg = f"Video key '{video_key}'s horizon must be {expected_t}. Got {arr.shape[1]}"
            raise ValueError(msg)
        if arr.shape[-1] != 3:
            msg = f"Video key '{video_key}' must have 3 channels, got {arr.shape[-1]}"
            raise ValueError(msg)

    # State validation
    state_obs = observation["state"]
    for state_key in STATE_KEYS:
        if state_key not in state_obs:
            msg = f"State key '{state_key}' must be in observation"
            raise ValueError(msg)
        arr = state_obs[state_key]
        if not isinstance(arr, np.ndarray):
            msg = f"State key '{state_key}' must be a numpy array"
            raise TypeError(msg)
        if arr.dtype != np.float32:
            msg = f"State key '{state_key}' must be float32, got {arr.dtype}"
            raise TypeError(msg)
        if arr.ndim != 3:
            msg = f"State key '{state_key}' must be (B, T, D), got shape {arr.shape}"
            raise ValueError(msg)

    # Language validation
    lang_obs = observation["language"]
    if LANGUAGE_KEY not in lang_obs:
        msg = f"Language key '{LANGUAGE_KEY}' must be in observation"
        raise ValueError(msg)


# ---------------------------------------------------------------------------
# Inference (stateless, time-based oscillation)
# ---------------------------------------------------------------------------


def _predict(observation: dict[str, Any], options: dict[str, Any] | None = None) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    """GR00T get_action: observation → (action_dict, info_dict)."""
    _check_observation(observation)

    state_obs = observation["state"]
    video_obs = observation["video"]
    language_obs = observation["language"]

    horizon = ACTION_HORIZON
    if options and "action_horizon" in options:
        horizon = int(options["action_horizon"])

    _service_state.record_request(
        state_keys=list(state_obs.keys()),
        video_keys=list(video_obs.keys()),
        language_keys=list(language_obs.keys()),
    )

    # Compute actions for each action key
    action: dict[str, np.ndarray] = {}
    for key in ACTION_KEYS:
        arr = state_obs[key]
        current = arr[0, -1, :]  # (D,) — last timestep, first batch
        action[key] = _generate_action(key, current, horizon)

    return action, {}


def _generate_action(key: str, current: np.ndarray, horizon: int) -> np.ndarray:
    """Generate absolute joint targets using time-based oscillation."""
    num_joints = current.shape[0]

    if "gripper" in key.lower():
        t0 = time.monotonic()
        result = np.zeros((1, horizon, 1), dtype=np.float32)
        for t in range(horizon):
            time_at_step = t0 + t * 0.033
            result[0, t, 0] = 50.0 + 50.0 * math.sin(0.5 * math.pi * time_at_step)
        return result

    # Arm joints: smooth oscillation
    phase = float(sum(ord(ch) for ch in key) % 360) * math.pi / 180.0
    t0 = time.monotonic()
    freq = 2.0
    dt_s = 0.033

    # Per-joint amplitude scaling (base still, wrist moves most)
    joint_scale = [0.0, 0.3, 0.3, 0.6, 1.0, 1.0]
    while len(joint_scale) < num_joints:
        joint_scale.append(1.0)

    result = np.zeros((1, horizon, num_joints), dtype=np.float32)
    for t in range(horizon):
        time_at_step = t0 + t * dt_s
        for j in range(num_joints):
            wave = math.sin(2.0 * math.pi * freq * time_at_step + phase + j * 0.7)
            result[0, t, j] = current[j] + AMPLITUDE * joint_scale[j] * wave
    return result


# ---------------------------------------------------------------------------
# Service state
# ---------------------------------------------------------------------------


class ServiceState:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._total_requests = 0
        self._last_request_at: float | None = None
        self._last_state_keys: list[str] = []
        self._last_video_keys: list[str] = []
        self._last_language_keys: list[str] = []

    def record_request(self, *, state_keys: list[str], video_keys: list[str], language_keys: list[str]) -> None:
        with self._lock:
            self._total_requests += 1
            self._last_request_at = time.monotonic()
            self._last_state_keys = list(state_keys)
            self._last_video_keys = list(video_keys)
            self._last_language_keys = list(language_keys)

    def snapshot(self, *, zmq_running: bool) -> dict[str, Any]:
        with self._lock:
            age = None
            if self._last_request_at is not None:
                age = round(time.monotonic() - self._last_request_at, 3)
            return {
                "service": "mock-groot-policy-service",
                "zmq_running": zmq_running,
                "total_requests": self._total_requests,
                "last_request_age_s": age,
                "state_keys": list(STATE_KEYS),
                "action_keys": list(ACTION_KEYS),
                "video_keys": list(VIDEO_KEYS),
                "action_horizon": ACTION_HORIZON,
                "last_state_keys": self._last_state_keys,
                "last_video_keys": self._last_video_keys,
            }


_service_state = ServiceState()


# ---------------------------------------------------------------------------
# ZMQ server
# ---------------------------------------------------------------------------


def _modality_config_envelope(config: dict[str, Any]) -> dict[str, Any]:
    """Wrap a modality config dict in the __ModalityConfig_class__ envelope (matches real server)."""
    return {"__ModalityConfig_class__": True, "as_json": config}


class ZmqPolicyServer(threading.Thread):
    def __init__(self, *, host: str, port: int, api_token: str | None) -> None:
        super().__init__(daemon=True, name="mock-groot-zmq")
        self._host = host
        self._port = port
        self._api_token = api_token
        self._stop_event = threading.Event()
        self._ready_event = threading.Event()

    def stop(self) -> None:
        self._stop_event.set()

    @property
    def is_ready(self) -> bool:
        return self._ready_event.is_set()

    def run(self) -> None:
        import zmq

        context = zmq.Context()
        socket = context.socket(zmq.REP)
        socket.bind(f"tcp://{self._host}:{self._port}")
        self._ready_event.set()
        logger.info("GR00T mock server listening on tcp://%s:%d", self._host, self._port)

        try:
            while not self._stop_event.is_set():
                if socket.poll(timeout=200) == 0:
                    continue
                message = socket.recv()
                response = self._handle_request(MsgSerializer.from_bytes(message))
                socket.send(MsgSerializer.to_bytes(response))
        finally:
            socket.close(linger=0)
            context.term()
            logger.info("GR00T mock server stopped")

    def _handle_request(self, request: Any) -> object:  # noqa: PLR0911
        try:
            if not isinstance(request, dict):
                return {"error": "Request must be a dict"}

            if self._api_token is not None and request.get("api_token") != self._api_token:
                return {"error": "Unauthorized: Invalid API token"}

            endpoint = request.get("endpoint", "get_action")

            if endpoint == "ping":
                return {"status": "ok", "message": "Server is running"}

            if endpoint == "kill":
                self._stop_event.set()
                return {"status": "ok", "message": "Server stopping"}

            if endpoint == "reset":
                return {}

            if endpoint == "get_modality_config":
                return {
                    "video": _modality_config_envelope({
                        "delta_indices": VIDEO_DELTA_INDICES,
                        "modality_keys": list(VIDEO_KEYS),
                        "sin_cos_embedding_keys": None,
                        "mean_std_embedding_keys": None,
                        "action_configs": None,
                    }),
                    "state": _modality_config_envelope({
                        "delta_indices": STATE_DELTA_INDICES,
                        "modality_keys": list(STATE_KEYS),
                        "sin_cos_embedding_keys": None,
                        "mean_std_embedding_keys": None,
                        "action_configs": None,
                    }),
                    "action": _modality_config_envelope({
                        "delta_indices": list(range(ACTION_HORIZON)),
                        "modality_keys": list(ACTION_KEYS),
                        "sin_cos_embedding_keys": None,
                        "mean_std_embedding_keys": None,
                        "action_configs": [
                            {"rep": "ABSOLUTE", "type": "NON_EEF", "format": "DEFAULT", "state_key": "left_arm"},
                            {"rep": "ABSOLUTE", "type": "NON_EEF", "format": "DEFAULT", "state_key": "right_arm"},
                            {"rep": "ABSOLUTE", "type": "NON_EEF", "format": "DEFAULT", "state_key": "left_gripper"},
                            {"rep": "ABSOLUTE", "type": "NON_EEF", "format": "DEFAULT", "state_key": "right_gripper"},
                        ],
                    }),
                    "language": _modality_config_envelope({
                        "delta_indices": [0],
                        "modality_keys": [LANGUAGE_KEY],
                        "sin_cos_embedding_keys": None,
                        "mean_std_embedding_keys": None,
                        "action_configs": None,
                    }),
                }

            if endpoint == "get_action":
                data = request.get("data", {})
                if not isinstance(data, dict):
                    return {"error": "get_action data must be a dict"}
                observation = data.get("observation")
                options = data.get("options")
                if not isinstance(observation, dict):
                    return {"error": "get_action requires a dict observation"}
                return _predict(observation, options)

            return {"error": f"Unknown endpoint: {endpoint}"}

        except Exception as exc:
            logger.exception("Request failed")
            return {"error": str(exc)}


# ---------------------------------------------------------------------------
# FastAPI (health/status only — inference is via ZMQ)
# ---------------------------------------------------------------------------

_zmq_server = ZmqPolicyServer(host=ZMQ_HOST, port=ZMQ_PORT, api_token=API_TOKEN)


@asynccontextmanager
async def lifespan(_: FastAPI) -> Iterator[None]:
    _zmq_server.start()
    yield
    _zmq_server.stop()
    _zmq_server.join(timeout=2.0)


app = FastAPI(
    title="Mock GR00T Policy Service (Dual-Arm UR10e)",
    version="0.3.0",
    description="Stateless mock GR00T server for dual-arm UR10e. Same ZMQ protocol as the real NVIDIA GR00T inference server.",
    root_path=BASE_PATH,
    docs_url="/",
    lifespan=lifespan,
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/health")
async def health() -> dict[str, str]:
    return {"status": "ok", "service": "mock-groot-policy-service"}


@app.get("/status")
async def status() -> dict[str, Any]:
    return _service_state.snapshot(zmq_running=_zmq_server.is_alive())


@app.get("/app_icon.png", include_in_schema=False)
async def app_icon() -> FileResponse:
    return FileResponse("static/app_icon.png", media_type="image/png")


def main() -> None:
    uvicorn.run(app, host="0.0.0.0", port=3000, log_level="info")  # noqa: S104


if __name__ == "__main__":
    main()
