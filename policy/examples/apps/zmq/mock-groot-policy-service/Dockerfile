FROM ghcr.io/astral-sh/uv:0.7.3 AS uv

FROM python:3.12-slim-bookworm

WORKDIR /app

COPY pyproject.toml ./

ENV UV_COMPILE_BYTECODE=1
ENV UV_LINK_MODE=copy

RUN --mount=from=uv,source=/uv,target=/bin/uv \
    --mount=type=cache,target=/root/.cache/uv \
    uv sync --no-install-project --no-dev

COPY static static/
COPY mock_groot_policy_service mock_groot_policy_service/

RUN --mount=from=uv,source=/uv,target=/bin/uv \
    --mount=type=cache,target=/root/.cache/uv \
    uv sync --no-dev

ENV PATH="/app/.venv/bin:$PATH"

ENTRYPOINT ["python", "-m", "mock_groot_policy_service"]
