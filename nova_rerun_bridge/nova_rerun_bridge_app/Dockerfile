FROM python:3.11-slim AS builder

RUN pip install --upgrade pip \
    && pip install poetry==2.0.1

ENV POETRY_NO_INTERACTION=1 \
    POETRY_VIRTUALENVS_IN_PROJECT=1 \
    POETRY_VIRTUALENVS_CREATE=1 \
    POETRY_CACHE_DIR=/tmp/poetry_cache

WORKDIR /app

# Copy entire project first
COPY . .

# Install dependencies including the local package
RUN --mount=type=cache,target=$POETRY_CACHE_DIR poetry install --extras "nova-rerun-bridge"

FROM python:3.11-slim AS runtime

RUN apt-get update && apt-get install -y nginx gettext-base libnginx-mod-stream

ENV VIRTUAL_ENV=/app/.venv \
    PATH="/app/.venv/bin:$PATH"

COPY --from=builder ${VIRTUAL_ENV} ${VIRTUAL_ENV}

WORKDIR /app
COPY nginx.conf /etc/nginx/nginx.conf
COPY nginx.http.conf.template .
COPY static static
COPY models models
COPY data data
COPY nova nova
COPY nova_rerun_bridge nova_rerun_bridge
COPY start.sh /app/start.sh

RUN chmod +x /app/start.sh

ENTRYPOINT ["/app/start.sh"]