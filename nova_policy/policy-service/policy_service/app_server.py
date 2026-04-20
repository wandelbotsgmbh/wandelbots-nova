from __future__ import annotations

import os
from typing import TYPE_CHECKING

from fastapi import FastAPI, HTTPException, Response
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, HTMLResponse
import uvicorn

from policy_service.models import HealthzResponse, PolicyInfoResponse, PolicyRunResponse
from policy_service.runtime import MockPolicyRuntime, PolicyConflictError

if TYPE_CHECKING:
    from policy_service.models import PolicyStartRequest

BASE_PATH = os.getenv("BASE_PATH", "")

runtime = MockPolicyRuntime()
app = FastAPI(
    title="NOVA Policy Service",
    version="0.1.0",
    description="Mock policy control service as a NOVA-native app",
    root_path=BASE_PATH,
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/", summary="Open API explorer", response_class=HTMLResponse)
async def root() -> str:
    return f"""
    <!doctype html>
    <html lang=\"en\">
      <head>
        <meta charset=\"utf-8\">
        <meta name=\"viewport\" content=\"width=device-width, initial-scale=1, shrink-to-fit=no\">
        <title>NOVA Policy Service</title>
        <script src=\"https://unpkg.com/@stoplight/elements/web-components.min.js\"></script>
        <link rel=\"stylesheet\" href=\"https://unpkg.com/@stoplight/elements/styles.min.css\">
      </head>
      <body>
        <elements-api
          apiDescriptionUrl=\"{BASE_PATH}/openapi.json\"
          router=\"hash\"
          layout=\"sidebar\"
          tryItCredentialsPolicy=\"same-origin\"
        />
      </body>
    </html>
    """


@app.get("/app_icon.png", summary="Serve app icon")
async def get_app_icon() -> FileResponse:
    try:
        return FileResponse(path="static/app_icon.png", media_type="image/png")
    except FileNotFoundError as exc:
        raise HTTPException(status_code=404, detail="Icon not found") from exc


@app.get("/healthz", response_model=HealthzResponse)
async def healthz() -> HealthzResponse:
    return HealthzResponse(status="ok")


@app.get("/policies", response_model=list[PolicyInfoResponse])
async def list_policies() -> list[PolicyInfoResponse]:
    policies = runtime.list_policies()
    return [
        PolicyInfoResponse(policy=policy, loaded=True, app_state=runtime.app_state)
        for policy in policies
    ]


@app.get("/policies/{policy}", response_model=PolicyInfoResponse)
async def get_policy(policy: str) -> PolicyInfoResponse:
    loaded = runtime.loaded_policy == policy
    return PolicyInfoResponse(policy=policy, loaded=loaded, app_state=runtime.app_state)


@app.post("/policies/{policy}/start", response_model=PolicyRunResponse)
async def start_policy(policy: str, request: PolicyStartRequest) -> PolicyRunResponse:
    try:
        return await runtime.start(policy, request)
    except PolicyConflictError as exc:
        raise HTTPException(status_code=406, detail=str(exc)) from exc


@app.post("/policies/{policy}/stop", status_code=204)
async def stop_policy(policy: str, run: str | None = None) -> Response:
    await runtime.stop(policy, run)
    return Response(status_code=204)


@app.get("/policies/{policy}/runs/{run}", response_model=PolicyRunResponse)
async def get_policy_run(policy: str, run: str) -> PolicyRunResponse:
    try:
        return await runtime.get_run(policy, run)
    except KeyError as exc:
        raise HTTPException(status_code=404, detail="Run not found") from exc


def main(host: str = "127.0.0.1", port: int = 3000) -> None:
    uvicorn.run(
        app,
        host=host,
        port=port,
        reload=False,
        log_level="info",
        proxy_headers=True,
        forwarded_allow_ips="*",
    )


if __name__ == "__main__":
    main()
