"""Mock policy service — simulates an inference server.

Provides:
- POST /start — start the policy (begins generating actions when clients connect)
- POST /stop — stop the policy
- GET /status — check if policy is running
- WS /predict — single-step action stream (connect after /start)
- WS /predict_chunked — multi-step action stream (16 steps at 33ms)
"""

import logging
import math
import time

import uvicorn
from decouple import config
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from pydantic import BaseModel, Field

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

BASE_PATH = config("BASE_PATH", default="", cast=str)

app = FastAPI(
    title="Mock Policy Service",
    version="0.1.0",
    description="Simulates a policy inference server. Start the policy via POST /start, then connect via WebSocket.",
    root_path=BASE_PATH,
    docs_url="/",
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# ---------------------------------------------------------------------------
# Policy state
# ---------------------------------------------------------------------------


class PolicyConfig(BaseModel):
    amplitude: float = Field(default=0.08, description="Oscillation amplitude in radians")
    frequency: float = Field(default=0.3, description="Oscillation frequency in Hz")
    duration_s: float = Field(
        default=30.0, description="Auto-stop after this many seconds (0=infinite)"
    )


class PolicyState:
    def __init__(self):
        self.running: bool = False
        self.config: PolicyConfig = PolicyConfig()
        self.start_time: float = 0.0
        self.connections: int = 0

    def start(self, cfg: PolicyConfig):
        self.running = True
        self.config = cfg
        self.start_time = time.monotonic()
        logger.info(
            "Policy started: amplitude=%.3f, frequency=%.2f, duration=%.0fs",
            cfg.amplitude,
            cfg.frequency,
            cfg.duration_s,
        )

    def stop(self):
        self.running = False
        logger.info("Policy stopped")

    @property
    def elapsed(self) -> float:
        return time.monotonic() - self.start_time if self.running else 0.0

    @property
    def is_expired(self) -> bool:
        if self.config.duration_s <= 0:
            return False
        return self.elapsed >= self.config.duration_s


policy_state = PolicyState()


# ---------------------------------------------------------------------------
# REST endpoints
# ---------------------------------------------------------------------------


@app.get("/health")
async def health():
    return {"status": "ok", "service": "mock-policy-service"}


@app.get("/app_icon.png", include_in_schema=False)
async def app_icon():
    return FileResponse("static/app_icon.png", media_type="image/png")


class StatusResponse(BaseModel):
    running: bool
    elapsed_s: float
    connections: int
    config: PolicyConfig


@app.get("/status", response_model=StatusResponse)
async def status():
    """Get the current policy state."""
    return StatusResponse(
        running=policy_state.running,
        elapsed_s=round(policy_state.elapsed, 1),
        connections=policy_state.connections,
        config=policy_state.config,
    )


@app.post("/start", response_model=StatusResponse)
async def start_policy(cfg: PolicyConfig = PolicyConfig()):
    """Start the policy. Connected WebSocket clients will immediately receive actions.

    The robot controller should already be running and waiting (READY phase).
    Starting the policy is what triggers actual robot motion.
    """
    policy_state.start(cfg)
    return StatusResponse(
        running=True, elapsed_s=0.0, connections=policy_state.connections, config=cfg
    )


@app.post("/stop", response_model=StatusResponse)
async def stop_policy():
    """Stop the policy. Active WebSocket connections will receive {"done": true}."""
    policy_state.stop()
    return StatusResponse(running=False, elapsed_s=0.0, connections=0, config=policy_state.config)


# ---------------------------------------------------------------------------
# WebSocket endpoints
# ---------------------------------------------------------------------------


def generate_target(home: list[float], t: float, cfg: PolicyConfig) -> list[float]:
    """Generate sinusoidal joint target."""
    return [
        home[i] + cfg.amplitude * math.sin(2.0 * math.pi * cfg.frequency * t + i * 0.4)
        for i in range(len(home))
    ]


@app.websocket("/predict")
async def predict(ws: WebSocket):
    """Single-step prediction. Responds with one joint target per observation.

    If the policy is not yet started, the connection stays open and waits.
    Once the policy starts, actions flow immediately.

    Protocol:
        Client sends: {"joints": [j1..j6], "motion_group_id": "0@ur10e"}
        Server sends: {"joints": {"0@ur10e": [[j1..j6]]}, "dt_ms": 0}
        Server sends: {"done": true} when policy stops/expires
    """
    await ws.accept()
    policy_state.connections += 1
    logger.info("Client connected to /predict (total: %d)", policy_state.connections)

    home: list[float] | None = None
    mg_id: str = "0@ur10e"
    was_executing: bool = False

    try:
        import json

        while True:
            raw = await ws.receive_text()
            obs = json.loads(raw)

            # Handle executor stop notification
            if obs.get("executor_stopped"):
                reason = obs.get("reason", "unknown")
                logger.info("Executor stopped (reason=%s) for %s", reason, mg_id)
                policy_state.stop()
                break

            if home is None:
                home = obs["joints"]
                mg_id = obs.get("motion_group_id", "0@ur10e")

            # Check if policy is active
            if not policy_state.running or policy_state.is_expired:
                if was_executing:
                    # Transition: was executing → now done. Signal episode end.
                    was_executing = False
                    await ws.send_text(json.dumps({"done": True}))
                    continue
                # Not running: signal waiting (no action)
                await ws.send_text(json.dumps({"waiting": True}))
                continue

            was_executing = True

            t = policy_state.elapsed
            target = generate_target(home, t, policy_state.config)

            await ws.send_text(json.dumps({"joints": {mg_id: [target]}, "dt_ms": 0}))

    except WebSocketDisconnect:
        pass
    except Exception as e:
        logger.error("Predict error: %s", e)
    finally:
        policy_state.connections -= 1
        logger.info("Client disconnected from /predict (total: %d)", policy_state.connections)


@app.websocket("/predict_chunked")
async def predict_chunked(ws: WebSocket):
    """Multi-step prediction (ACT/Pi0 style). Outputs 16-step chunks."""
    await ws.accept()
    policy_state.connections += 1
    logger.info("Client connected to /predict_chunked")

    home: list[float] | None = None
    mg_id: str = "0@ur10e"
    chunk_size = 16
    dt_ms = 33.0

    try:
        import json

        while True:
            raw = await ws.receive_text()

            if not policy_state.running or policy_state.is_expired:
                await ws.send_text(json.dumps({"done": True}))
                break

            obs = json.loads(raw)
            if home is None:
                home = obs["joints"]
                mg_id = obs.get("motion_group_id", "0@ur10e")

            t_base = policy_state.elapsed
            steps = [
                generate_target(home, t_base + step * (dt_ms / 1000.0), policy_state.config)
                for step in range(chunk_size)
            ]

            await ws.send_text(json.dumps({"joints": {mg_id: steps}, "dt_ms": dt_ms}))

    except WebSocketDisconnect:
        pass
    except Exception as e:
        logger.error("Chunked predict error: %s", e)
    finally:
        policy_state.connections -= 1


def main():
    uvicorn.run(app, host="0.0.0.0", port=3000, log_level="info")  # noqa: S104


if __name__ == "__main__":
    main()
