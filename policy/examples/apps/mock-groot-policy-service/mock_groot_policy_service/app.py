"""Mock GR00T policy service.

This app exposes a GR00T-compatible ZMQ REQ/REP inference endpoint and a small
HTTP API for health and status. Predictions are intentionally stateless:
identical observations produce identical action chunks.
"""

from __future__ import annotations

import io
import logging
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
from pydantic import BaseModel, Field

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

BASE_PATH = config("BASE_PATH", default="", cast=str)
ZMQ_HOST = config("ZMQ_HOST", default="0.0.0.0", cast=str)  # noqa: S104
ZMQ_PORT = config("ZMQ_PORT", default=5555, cast=int)
API_TOKEN = config("GROOT_API_TOKEN", default=None, cast=str)

STATE_KEYS = ("left_arm", "right_arm", "left_gripper", "right_gripper")
LANGUAGE_KEY = "task"


class MockConfig(BaseModel):
    action_horizon: int = Field(default=16, ge=1, description="Returned action chunk length")
    delta_scale: float = Field(default=0.025, ge=0.0, description="Joint oscillation amplitude")
    step_phase: float = Field(default=0.35, ge=0.0, description="Phase added per predicted step")
    joint_phase: float = Field(default=0.17, ge=0.0, description="Phase added per joint index")


class StatusResponse(BaseModel):
    service: str
    stateless_inference: bool
    zmq_host: str
    zmq_port: int
    zmq_running: bool
    total_requests: int
    last_request_age_s: float | None
    expected_state_keys: list[str]
    last_state_keys: list[str]
    last_video_keys: list[str]
    last_language_keys: list[str]
    config: MockConfig


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


class ServiceState:
    def __init__(self, cfg: MockConfig) -> None:
        self._cfg = cfg
        self._lock = threading.Lock()
        self._total_requests = 0
        self._last_request_at: float | None = None
        self._last_state_keys: list[str] = []
        self._last_video_keys: list[str] = []
        self._last_language_keys: list[str] = []

    @property
    def config(self) -> MockConfig:
        return self._cfg

    def record_request(
        self,
        *,
        state_keys: list[str],
        video_keys: list[str],
        language_keys: list[str],
    ) -> None:
        with self._lock:
            self._total_requests += 1
            self._last_request_at = time.monotonic()
            self._last_state_keys = list(state_keys)
            self._last_video_keys = list(video_keys)
            self._last_language_keys = list(language_keys)

    def snapshot(self, *, zmq_running: bool) -> StatusResponse:
        with self._lock:
            age = None
            if self._last_request_at is not None:
                age = round(time.monotonic() - self._last_request_at, 3)
            return StatusResponse(
                service="mock-groot-policy-service",
                stateless_inference=True,
                zmq_host=ZMQ_HOST,
                zmq_port=ZMQ_PORT,
                zmq_running=zmq_running,
                total_requests=self._total_requests,
                last_request_age_s=age,
                expected_state_keys=list(STATE_KEYS),
                last_state_keys=list(self._last_state_keys),
                last_video_keys=list(self._last_video_keys),
                last_language_keys=list(self._last_language_keys),
                config=self._cfg,
            )


class MockGr00tPolicy:
    def __init__(self, state: ServiceState) -> None:
        self._state = state

    def get_action(
        self,
        observation: dict[str, Any],
        options: dict[str, Any] | None = None,
    ) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
        state_obs = observation.get("state")
        if not isinstance(state_obs, dict):
            msg = "Observation must contain a 'state' dict"
            raise ValueError(msg)

        language_obs = observation.get("language")
        if not isinstance(language_obs, dict) or LANGUAGE_KEY not in language_obs:
            msg = "Observation must contain language.task"
            raise ValueError(msg)

        cfg = self._state.config
        horizon = cfg.action_horizon
        delta_scale = cfg.delta_scale
        if options is not None:
            if "action_horizon" in options:
                horizon = int(options["action_horizon"])
            if "delta_scale" in options:
                delta_scale = float(options["delta_scale"])

        action: dict[str, np.ndarray] = {}
        for key in STATE_KEYS:
            if key not in state_obs:
                msg = f"Missing required state key '{key}'"
                raise ValueError(msg)
            arr = np.asarray(state_obs[key], dtype=np.float32)
            if arr.ndim != 3:
                msg = f"State key '{key}' must have shape (B, T, D), got {arr.shape}"
                raise ValueError(msg)
            current = arr[:, -1, :]
            action[key] = self._generate_action(
                key,
                current,
                horizon,
                delta_scale,
                cfg.step_phase,
                cfg.joint_phase,
            )

        video_obs = observation.get("video", {})
        self._state.record_request(
            state_keys=list(state_obs.keys()),
            video_keys=list(video_obs.keys()) if isinstance(video_obs, dict) else [],
            language_keys=list(language_obs.keys()),
        )
        info = {
            "stateless": True,
            "action_horizon": horizon,
            "action_keys": list(action.keys()),
            "dt_ms": 33.0,
        }
        return action, info

    def reset(self, options: dict[str, Any] | None = None) -> dict[str, Any]:
        del options
        return {"stateless": True}

    def get_modality_config(self) -> dict[str, Any]:
        return {
            "state": {"delta_indices": [0], "modality_keys": list(STATE_KEYS)},
            "action": {
                "delta_indices": list(range(self._state.config.action_horizon)),
                "modality_keys": list(STATE_KEYS),
            },
            "language": {"delta_indices": [0], "modality_keys": [LANGUAGE_KEY]},
        }

    @staticmethod
    def _generate_action(
        key: str,
        current: np.ndarray,
        horizon: int,
        delta_scale: float,
        step_phase: float,
        joint_phase: float,
    ) -> np.ndarray:
        repeated = np.repeat(current[:, np.newaxis, :], horizon, axis=1).astype(np.float32)
        phase = np.float32((sum(ord(ch) for ch in key) % 360) * np.pi / 180.0)
        step_offsets = np.arange(horizon, dtype=np.float32)[:, np.newaxis] * np.float32(step_phase)
        lowered = key.lower()
        if "gripper" in lowered or "hand" in lowered:
            wave = np.sin(step_offsets + phase, dtype=np.float32)
            gripper = np.where(wave[np.newaxis, :, :] >= 0.0, 100.0, 0.0)
            return gripper.astype(np.float32)

        joint_offsets = np.arange(current.shape[1], dtype=np.float32)[np.newaxis, :] * np.float32(
            joint_phase
        )
        wave = np.sin(step_offsets + joint_offsets + phase, dtype=np.float32)
        return repeated + (np.float32(delta_scale) * wave[np.newaxis, :, :])


class ZmqPolicyServer(threading.Thread):
    def __init__(
        self,
        *,
        policy: MockGr00tPolicy,
        host: str,
        port: int,
        api_token: str | None,
    ) -> None:
        super().__init__(daemon=True, name="mock-groot-zmq")
        self._policy = policy
        self._host = host
        self._port = port
        self._api_token = api_token
        self._stop_event = threading.Event()
        self._ready_event = threading.Event()
        self._context: Any | None = None
        self._socket: Any | None = None

    def stop(self) -> None:
        self._stop_event.set()

    @property
    def is_ready(self) -> bool:
        return self._ready_event.is_set()

    def run(self) -> None:
        import zmq

        self._context = zmq.Context()
        self._socket = self._context.socket(zmq.REP)
        self._socket.bind(f"tcp://{self._host}:{self._port}")
        self._ready_event.set()
        logger.info("Mock GR00T ZMQ server listening on tcp://%s:%d", self._host, self._port)

        try:
            while not self._stop_event.is_set():
                if self._socket.poll(timeout=200) == 0:
                    continue
                message = self._socket.recv()
                response = self._handle_request(MsgSerializer.from_bytes(message))
                self._socket.send(MsgSerializer.to_bytes(response))
        finally:
            if self._socket is not None:
                self._socket.close(linger=0)
            if self._context is not None:
                self._context.term()
            logger.info("Mock GR00T ZMQ server stopped")

    def _handle_request(self, request: Any) -> object:
        try:
            if not isinstance(request, dict):
                msg = "Request must be a dict"
                raise TypeError(msg)
            if self._api_token is not None and request.get("api_token") != self._api_token:
                return {"error": "Unauthorized: Invalid API token"}

            endpoint = request.get("endpoint", "get_action")
            result: object
            if endpoint == "ping":
                result = {"status": "ok", "message": "Server is running"}
            elif endpoint == "kill":
                self._stop_event.set()
                result = {"status": "ok", "message": "Server stopping"}
            elif endpoint == "reset":
                data = request.get("data", {})
                options = data.get("options") if isinstance(data, dict) else None
                result = self._policy.reset(options)
            elif endpoint == "get_modality_config":
                result = self._policy.get_modality_config()
            elif endpoint == "get_action":
                data = request.get("data", {})
                if not isinstance(data, dict):
                    msg = "get_action data must be a dict"
                    raise TypeError(msg)
                observation = data.get("observation")
                options = data.get("options")
                if not isinstance(observation, dict):
                    msg = "get_action requires a dict observation"
                    raise TypeError(msg)
                result = self._policy.get_action(observation, options)
            else:
                msg = f"Unknown endpoint: {endpoint}"
                raise ValueError(msg)
            return result
        except Exception as exc:
            logger.exception("Mock GR00T request failed")
            return {"error": str(exc)}


service_state = ServiceState(MockConfig())
service_policy = MockGr00tPolicy(service_state)
zmq_server = ZmqPolicyServer(
    policy=service_policy,
    host=ZMQ_HOST,
    port=ZMQ_PORT,
    api_token=API_TOKEN,
)


@asynccontextmanager
async def lifespan(_: FastAPI) -> Iterator[None]:
    zmq_server.start()
    yield
    zmq_server.stop()
    zmq_server.join(timeout=2.0)


app = FastAPI(
    title="Mock GR00T Policy Service",
    version="0.1.0",
    description="Stateless mock GR00T-compatible inference service.",
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


@app.get("/status", response_model=StatusResponse)
async def status() -> StatusResponse:
    return service_state.snapshot(zmq_running=zmq_server.is_alive())


@app.get("/app_icon.png", include_in_schema=False)
async def app_icon() -> FileResponse:
    return FileResponse("static/app_icon.png", media_type="image/png")


def main() -> None:
    uvicorn.run(
        app,
        host="0.0.0.0",  # noqa: S104
        port=3000,
        log_level="info",
        proxy_headers=True,
        forwarded_allow_ips="*",
    )


if __name__ == "__main__":
    main()
