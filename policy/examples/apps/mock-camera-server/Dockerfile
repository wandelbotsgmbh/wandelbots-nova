FROM ghcr.io/astral-sh/uv:0.7.3 AS uv
FROM python:3.12-slim-bookworm
WORKDIR /app
COPY pyproject.toml ./
ENV UV_COMPILE_BYTECODE=1 UV_LINK_MODE=copy
RUN --mount=from=uv,source=/uv,target=/bin/uv \
    --mount=type=cache,target=/root/.cache/uv \
    uv sync --no-install-project --no-dev
ENV PATH="/app/.venv/bin:$PATH"
RUN python -c "\
import shutil, os; from huggingface_hub import hf_hub_download; \
os.makedirs('videos', exist_ok=True); \
[shutil.copy(hf_hub_download('StefanWagnerWandelbots/pusht_physical_15fps', \
  f'videos/observation.images.{cam}/chunk-000/file-000.mp4', repo_type='dataset'), \
  f'videos/{cam}.mp4') for cam in ('flange', 'left', 'right')]"
COPY static static/
COPY mock_camera_server mock_camera_server/
RUN --mount=from=uv,source=/uv,target=/bin/uv --mount=type=cache,target=/root/.cache/uv uv sync --locked --no-dev
EXPOSE 9100
ENTRYPOINT ["python", "-m", "mock_camera_server"]
