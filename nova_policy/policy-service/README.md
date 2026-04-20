# policy_service

NOVA-native mock policy service app.

This app is generated from `nova app create policy-service -g python_app` and then adapted to provide
mock policy lifecycle and run-control endpoints for SDK integration work.

## Run locally

```bash
cd policy-service
uv run python -m policy_service
```

Server starts on `http://localhost:3000` by default.

## Mock API

- `GET /healthz`
- `GET /policies`
- `GET /policies/{policy}`
- `POST /policies/{policy}/start`
- `POST /policies/{policy}/stop`
- `GET /policies/{policy}/runs/{run}`

## URL note for SDK integration

Current mock routes are rooted at `/policies/...`.

When deployed as a NOVA app, a base path may be injected by platform/proxy. Keep SDK `policy_api_url`
configurable until deployed routing conventions are finalized.
