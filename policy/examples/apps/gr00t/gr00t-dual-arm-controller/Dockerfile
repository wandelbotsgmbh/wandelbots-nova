FROM ghcr.io/astral-sh/uv:0.7.3 AS uv

FROM python:3.12-slim-bookworm

RUN apt-get update && apt-get install -y --no-install-recommends git libxcb1 libgl1 && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY pyproject.toml ./

ENV UV_COMPILE_BYTECODE=1
ENV UV_LINK_MODE=copy

ARG CACHEBUST=1
RUN --mount=from=uv,source=/uv,target=/bin/uv \
    uv sync --no-install-project --no-dev --refresh

COPY static static/
COPY gr00t_dual_arm_controller gr00t_dual_arm_controller/

RUN --mount=from=uv,source=/uv,target=/bin/uv \
    uv sync --no-dev

ENV PATH="/app/.venv/bin:$PATH"

ENTRYPOINT ["python", "-m", "gr00t_dual_arm_controller"]
