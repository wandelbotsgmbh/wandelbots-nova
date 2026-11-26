"""
Example: a new policy provider in one class, for a policy served over HTTP.

The provider POSTs the observation as JSON and reads ``{"actions": {key: [[...], ...]}}`` back.
The schema builds the observation and decodes the answer; the executor places and streams it.
Nothing outside ``HttpPolicyClient`` is specific to this provider.

A mock server is included: it answers with a 15-step plan that swings joint 0.

Prerequisites:
    NOVA_API=http://<instance-ip>

Run (a throwaway virtual UR10e, continuous execution for 10 s):
    PYTHONPATH=. python novapolicy/examples/http_policy.py
"""

from __future__ import annotations

import asyncio
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import threading
import time
from typing import Any

import httpx

from novapolicy import ActionChunk, PolicyClient, PolicySchema, resolve_control_freq


class HttpPolicyClient(PolicyClient):
    """A policy behind an HTTP endpoint: the observation goes out as JSON, actions come back."""

    def __init__(
        self, url: str, *, control_freq: float | None = None, timeout_s: float = 5.0
    ) -> None:
        self._url = url
        self._timeout_s = timeout_s
        self._dt_ms = 1000.0 / resolve_control_freq(control_freq, owner="HttpPolicyClient")
        self._http: httpx.AsyncClient | None = None

    async def connect(self, motion_group_ids: list[str]) -> None:
        self._http = httpx.AsyncClient(timeout=self._timeout_s)

    async def get_actions(
        self,
        states: dict[str, Any],
        schema: PolicySchema,
        images: dict[str, Any] | None = None,
        io_values: dict[str, object] | None = None,
    ) -> ActionChunk:
        assert self._http is not None, "connect() first"
        observation = await schema.build_observation(states, io_values)
        groups = schema.observation_groups(observation)  # {"arm": [6 values], ...}
        reply = await self._http.post(self._url, json={"observation": groups})
        reply.raise_for_status()
        return schema.decode_actions(reply.json()["actions"], dt_ms=self._dt_ms, source="HTTP")

    async def close(self) -> None:
        if self._http is not None:
            await self._http.aclose()
            self._http = None


# -- A mock policy server, for this example and the tests -------------------------------------


def serve_mock_policy(
    center: list[float], *, key: str = "arm", steps: int = 15, control_freq: float = 30.0
) -> ThreadingHTTPServer:
    """Serve on a free local port (``server.server_address``) in a background thread: every
    POST answers with ``steps`` steps of joint 0 swinging 0.25 rad around ``center``, planned
    from the server's clock, which starts at the first request: the swing starts at ``center``,
    where the robot is. ``server.shutdown()`` stops it."""
    t0: list[float] = []
    first_request = threading.Lock()

    class Handler(BaseHTTPRequestHandler):
        def do_POST(self) -> None:
            self.rfile.read(int(self.headers["Content-Length"]))
            with first_request:
                if not t0:
                    t0.append(time.monotonic())
            now = time.monotonic() - t0[0]
            plan = [
                [
                    center[0] + 0.25 * math.sin(2 * math.pi * 0.4 * (now + k / control_freq)),
                    *center[1:],
                ]
                for k in range(steps)
            ]
            body = json.dumps({"actions": {key: plan}}).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, format: str, *args: Any) -> None:
            pass

    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server


async def main() -> None:
    import contextlib
    import uuid

    from nova import Nova, api
    from nova.cell import virtual_controller
    from novapolicy import Action, ContinuousExecution, Observation, PolicyExecutor

    start = [0.0, -1.2, 1.4, -1.8, -1.57, 0.0]
    server = serve_mock_policy(start)
    name = f"http-policy-{uuid.uuid4().hex[:6]}"
    async with Nova() as nova:
        cell = nova.cell()
        controller = await cell.ensure_controller(
            virtual_controller(
                name=name,
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type="universalrobots-ur10e",
            )
        )
        try:
            async with controller[0] as mg:
                schema = PolicySchema(
                    observations=[Observation.joint_positions("arm", source=mg, action=False)],
                    actions=[Action.joint_positions("arm", target=mg, mode="absolute")],
                )
                host, port = server.server_address[:2]
                result = await PolicyExecutor(
                    schema,
                    HttpPolicyClient(f"http://{host}:{port}/act", control_freq=30),
                    execution=ContinuousExecution(rate_hz=10),
                    n_action_steps=0,
                    start_joint_position={mg: start},
                    timeout_s=10.0,
                ).run()
                print("stopped:", result.reason)
        finally:
            server.shutdown()
            with contextlib.suppress(Exception):
                await cell.delete_robot_controller(name, timeout=25)


if __name__ == "__main__":
    asyncio.run(main())
