FROM ghcr.io/astral-sh/uv:0.7.3 AS uv

# Docker image with uv
# https://github.com/astral-sh/uv-docker-example/blob/main/Dockerfile
FROM python:3.12-slim-bookworm

WORKDIR /app

COPY pyproject.toml ./

# Enable bytecode compilation
ENV UV_COMPILE_BYTECODE=1
# Copy from the cache instead of linking since it's a mounted volume
ENV UV_LINK_MODE=copy

# Install the project's dependencies using the lockfile and settings
RUN --mount=from=uv,source=/uv,target=/bin/uv \
    --mount=type=cache,target=/root/.cache/uv \
    --mount=type=bind,source=pyproject.toml,target=pyproject.toml \
    uv sync --no-install-project --no-dev

COPY static static
COPY README.md ./
COPY app app

# Set INSTALL_DEV=true (e.g. via Skaffold) to include the dev dependency group
# (adds watchfiles, which powers uvicorn --reload for the hot-reload dev loop).
ARG INSTALL_DEV=false
RUN --mount=from=uv,source=/uv,target=/bin/uv \
    --mount=type=cache,target=/root/.cache/uv \
    sh -c 'if [ "$INSTALL_DEV" = "true" ]; then uv sync --locked; else uv sync --locked --no-dev; fi'

# Place executables in the environment at the front of the path
ENV PATH="/app/.venv/bin:$PATH"

ENTRYPOINT ["python", "-m", "app"]