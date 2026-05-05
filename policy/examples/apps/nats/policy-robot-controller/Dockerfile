FROM ghcr.io/astral-sh/uv:0.7.3 AS uv

FROM python:3.12-slim-bookworm

WORKDIR /app

COPY pyproject.toml ./

ENV UV_COMPILE_BYTECODE=1
ENV UV_LINK_MODE=copy

RUN --mount=from=uv,source=/uv,target=/bin/uv \
    --mount=type=cache,target=/root/.cache/uv \
    --mount=type=bind,source=pyproject.toml,target=pyproject.toml \
    uv sync --no-install-project --no-dev

# policy is copied from the repo root before building (see README)
COPY policy policy/

COPY static static/
COPY policy_robot_controller policy_robot_controller/

RUN --mount=from=uv,source=/uv,target=/bin/uv \
    --mount=type=cache,target=/root/.cache/uv \
    uv sync --locked --no-dev

ENV PATH="/app/.venv/bin:$PATH"

ENTRYPOINT ["python", "-m", "policy_robot_controller"]
