from __future__ import annotations

import asyncio
import contextlib
import logging
import os
from typing import Protocol, cast
from urllib.parse import urlparse

import httpx
from nova import Nova
from nova.cell.motion_group import MotionGroup
from nova.config import NOVA_ACCESS_TOKEN, NOVA_API, NovaConfig
from nova_policy import PolicyExecutionOptions, PolicyRunState, enable_motion_group_policy_extension

POLICY_SERVICE_URL = os.getenv(
    "POLICY_SERVICE_URL",
    "https://nova-policy-service.ai.gpucluster-dev.wandelbots.io",
)
POLICY_PATH = os.getenv(
    "POLICY_PATH",
    "StefanWagnerWandelbots/act_virtual_teleop_pickplace_easy",
)
CONTROLLER_NAME = os.getenv("NOVA_CONTROLLER", "ur10e")
MOTION_GROUP_INDEX = int(os.getenv("NOVA_MOTION_GROUP", "0"))
TASK = os.getenv("POLICY_TASK", "pick the cube and place it in the box")
TCP = os.getenv("NOVA_TCP")
MAX_OBSERVATIONS = int(os.getenv("MAX_OBSERVATIONS", "3"))
TIMEOUT_S = float(os.getenv("POLICY_TIMEOUT_S", "20"))
EXECUTE_ACTIONS = os.getenv("EXECUTE_ACTIONS", "false").lower() in {"1", "true", "yes"}
ALLOW_MOCK_IMAGES = os.getenv("ALLOW_MOCK_IMAGES", "true").lower() in {"1", "true", "yes"}
USE_GRIPPER = os.getenv("USE_GRIPPER", "true").lower() in {"1", "true", "yes"}
NATS_CONNECT_TIMEOUT_S = float(os.getenv("NATS_CONNECT_TIMEOUT_S", "5"))
NATS_MAX_RECONNECT_ATTEMPTS = int(os.getenv("NATS_MAX_RECONNECT_ATTEMPTS", "0"))
NATS_RECONNECT_WAIT_S = float(os.getenv("NATS_RECONNECT_WAIT_S", "1"))
NOVA_OPEN_TIMEOUT_S = float(os.getenv("NOVA_OPEN_TIMEOUT_S", "10"))

logger = logging.getLogger(__name__)


enable_motion_group_policy_extension()


class _HasModelDump(Protocol):
    def model_dump(self, *, mode: str, exclude_none: bool) -> dict[str, object]: ...


async def main() -> None:
    await _preflight_policy_service()

    if EXECUTE_ACTIONS:
        logger.warning(
            "EXECUTE_ACTIONS=true will command NOVA jogging velocities. "
            "Use only against a virtual/safe controller with validated limits."
        )

    config = _nova_config_from_env()
    logger.info("Connecting to NOVA_API=%s NATS=%s", config.host, _redact_nats_servers(config))
    await _preflight_nats_tcp(config)

    nova = Nova(config)
    try:
        await asyncio.wait_for(nova.open(), timeout=NOVA_OPEN_TIMEOUT_S)
        await _run_validation(nova)
    except TimeoutError as exc:
        raise RuntimeError(
            f"Timed out connecting to NOVA/NATS after {NOVA_OPEN_TIMEOUT_S}s. "
            "Check NOVA_API/NATS_BROKER reachability or VPN before running realtime validation."
        ) from exc
    finally:
        with contextlib.suppress(Exception):
            await nova.nats.close()
        await nova.close()


async def _run_validation(nova: Nova) -> None:
    controller = await nova.cell().controller(CONTROLLER_NAME)
    motion_group: MotionGroup = controller[MOTION_GROUP_INDEX]
    tcp = TCP or await motion_group.active_tcp_name() or "flange"
    motion_group_setup = await motion_group.get_setup(tcp)

    options = PolicyExecutionOptions(
        tcp=tcp,
        policy_api_url=POLICY_SERVICE_URL,
        realtime=True,
        execute_actions=EXECUTE_ACTIONS,
        max_observations=MAX_OBSERVATIONS,
        low_water_mark=1,
        allow_mock_images=ALLOW_MOCK_IMAGES,
        use_gripper=USE_GRIPPER,
        motion_group_setup=cast("_HasModelDump", motion_group_setup),
        joint_velocity_limit=float(os.getenv("JOINT_VELOCITY_LIMIT", "0.25")),
        joint_position_gain=float(os.getenv("JOINT_POSITION_GAIN", "1.0")),
        joint_position_tolerance=float(os.getenv("JOINT_POSITION_TOLERANCE", "0.01")),
        setup_velocity_limit_scale=float(os.getenv("SETUP_VELOCITY_LIMIT_SCALE", "0.1")),
    )

    async for state in motion_group.stream_policy(
        policy_path=POLICY_PATH,
        task=TASK,
        timeout_s=TIMEOUT_S,
        options=options,
    ):
        _log_state(state)


async def _preflight_nats_tcp(config: NovaConfig) -> None:
    nats_config = config.nats_client_config or {}
    servers = nats_config.get("servers")
    server = servers[0] if isinstance(servers, list) and servers else servers
    if not isinstance(server, str):
        return

    parsed = urlparse(server)
    if parsed.scheme not in {"ws", "wss"} or parsed.hostname is None:
        return
    port = parsed.port or (443 if parsed.scheme == "wss" else 80)
    try:
        reader, writer = await asyncio.wait_for(
            asyncio.open_connection(parsed.hostname, port),
            timeout=NATS_CONNECT_TIMEOUT_S,
        )
    except TimeoutError as exc:
        raise RuntimeError(
            f"Timed out opening TCP connection to NATS endpoint {parsed.hostname}:{port} "
            f"after {NATS_CONNECT_TIMEOUT_S}s. Check NOVA_API/NATS_BROKER reachability or VPN."
        ) from exc
    except OSError as exc:
        raise RuntimeError(
            f"Could not open TCP connection to NATS endpoint {parsed.hostname}:{port}: {exc}"
        ) from exc
    writer.close()
    await writer.wait_closed()
    _ = reader


async def _preflight_policy_service() -> None:
    health_url = f"{POLICY_SERVICE_URL.rstrip('/')}/healthz"
    policy_url = f"{POLICY_SERVICE_URL.rstrip('/')}/policy"
    async with httpx.AsyncClient(timeout=10.0) as client:
        health = await client.get(health_url)
        health.raise_for_status()
        policy = await client.get(policy_url)
        policy.raise_for_status()
    logger.info("Policy service health=%s policy=%s", health.text, policy.text)


def _nova_config_from_env() -> NovaConfig:
    nats_client_config = {
        "connect_timeout": NATS_CONNECT_TIMEOUT_S,
        "max_reconnect_attempts": NATS_MAX_RECONNECT_ATTEMPTS,
        "reconnect_time_wait": NATS_RECONNECT_WAIT_S,
    }
    return NovaConfig(
        host=os.getenv("NOVA_API", NOVA_API),
        access_token=os.getenv("NOVA_ACCESS_TOKEN", NOVA_ACCESS_TOKEN),
        nats_client_config=nats_client_config,
    )


def _redact_nats_servers(config: NovaConfig) -> object:
    nats_config = config.nats_client_config or {}
    servers = nats_config.get("servers")
    if isinstance(servers, str):
        return _redact_url_userinfo(servers)
    if isinstance(servers, list):
        return [
            _redact_url_userinfo(server) if isinstance(server, str) else server
            for server in servers
        ]
    return servers


def _redact_url_userinfo(url: str) -> str:
    scheme_separator = "://"
    if scheme_separator not in url or "@" not in url:
        return url
    scheme, rest = url.split(scheme_separator, 1)
    _userinfo, host_and_path = rest.split("@", 1)
    return f"{scheme}{scheme_separator}<redacted>@{host_and_path}"


def _log_state(state: PolicyRunState) -> None:
    realtime_metadata = (state.metadata or {}).get("realtime")
    logger.info(
        "run=%s state=%s elapsed=%s realtime=%s",
        state.run,
        state.state,
        state.elapsed_s,
        realtime_metadata,
    )


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    try:
        asyncio.run(main())
    except RuntimeError as exc:
        logger.error("Validation failed: %s", exc)
        raise SystemExit(1) from exc
