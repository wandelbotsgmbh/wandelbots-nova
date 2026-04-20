from __future__ import annotations

import asyncio
from typing import TYPE_CHECKING, cast
from urllib.parse import quote

if TYPE_CHECKING:
    from collections.abc import AsyncIterator, Mapping

    from .models import JsonValue, PolicyRunPayload

import httpx

from .models import PolicyRun

HTTP_OK_STATUS = 200
HTTP_CONFLICT_STATUS = 406
HTTP_NO_CONTENT_STATUS = 204

TERMINAL_POLICY_RUN_STATES = {"STOPPED", "TIMED_OUT", "FAILED"}


def _is_terminal_state(state: str) -> bool:
    return state in TERMINAL_POLICY_RUN_STATES


class PolicyConflictError(RuntimeError):
    """Raised when a policy run is already active for the requested scope."""


class NovaLeRobotPolicyClient:
    def __init__(
        self,
        base_url: str,
        access_token: str | None = None,
        timeout_s: float = 60.0,
        status_poll_interval_s: float = 0.25,
    ) -> None:
        self._base_url = base_url.rstrip("/")
        self._timeout_s = timeout_s
        self._status_poll_interval_s = status_poll_interval_s
        self._headers = {"Authorization": f"Bearer {access_token}"} if access_token else {}

    async def start_run(self, policy: str, payload: Mapping[str, object]) -> PolicyRun:
        policy_id = quote(policy, safe="")
        response = await self._request(
            "POST",
            f"/policies/{policy_id}/start",
            json_body=payload,
            expected_status={HTTP_OK_STATUS},
        )
        return PolicyRun.from_dict(response)

    async def stop_run(self, policy: str, run: str | None = None) -> None:
        policy_id = quote(policy, safe="")
        params = {"run": run} if run else None
        await self._request(
            "POST",
            f"/policies/{policy_id}/stop",
            params=params,
            expected_status={HTTP_NO_CONTENT_STATUS},
        )

    async def get_run(self, policy: str, run: str) -> PolicyRun:
        policy_id = quote(policy, safe="")
        response = await self._request(
            "GET",
            f"/policies/{policy_id}/runs/{run}",
            expected_status={HTTP_OK_STATUS},
        )
        return PolicyRun.from_dict(response)

    async def stream_run(self, policy: str, run: str) -> AsyncIterator[PolicyRun]:
        while True:
            status = await self.get_run(policy=policy, run=run)
            yield status
            if _is_terminal_state(status.state):
                return
            await asyncio.sleep(self._status_poll_interval_s)

    async def _request(
        self,
        method: str,
        path: str,
        *,
        expected_status: set[int],
        json_body: Mapping[str, object] | None = None,
        params: Mapping[str, str] | None = None,
    ) -> PolicyRunPayload:
        async with httpx.AsyncClient(
            base_url=self._base_url,
            headers=self._headers,
            timeout=self._timeout_s,
        ) as client:
            response = await client.request(method, path, json=json_body, params=params)

        if response.status_code == HTTP_CONFLICT_STATUS:
            raise PolicyConflictError(response.text)

        if response.status_code not in expected_status:
            response.raise_for_status()

        if response.status_code == HTTP_NO_CONTENT_STATUS:
            return cast("PolicyRunPayload", {})

        payload_raw = cast("object", response.json())
        if not isinstance(payload_raw, dict):
            raise ValueError("Policy service response must be a JSON object")
        payload = cast("dict[str, object]", payload_raw)

        run = payload.get("run")
        state = payload.get("state")
        if not isinstance(run, str) or not isinstance(state, str):
            raise ValueError("Policy service response must contain string 'run' and 'state'")

        policy_value = payload.get("policy")
        policy_name = policy_value if isinstance(policy_value, str) else ""

        parsed: PolicyRunPayload = {
            "run": run,
            "state": state,
            "policy": policy_name,
        }

        start_time = payload.get("start_time")
        if isinstance(start_time, str):
            parsed["start_time"] = start_time

        timeout_s = payload.get("timeout_s")
        if isinstance(timeout_s, (int, float)):
            parsed["timeout_s"] = float(timeout_s)

        elapsed_s = payload.get("elapsed_s")
        if isinstance(elapsed_s, (int, float)):
            parsed["elapsed_s"] = float(elapsed_s)

        metadata = payload.get("metadata")
        if isinstance(metadata, dict):
            parsed["metadata"] = cast("dict[str, JsonValue]", metadata)

        return parsed
