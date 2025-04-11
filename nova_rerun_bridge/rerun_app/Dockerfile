FROM python:3.11-slim AS builder

RUN pip install --upgrade pip \
    && pip install uv

WORKDIR /app

# Copy entire project first
COPY . .

# Install dependencies including the local package
RUN uv sync --extra "nova-rerun-bridge"

FROM python:3.11-slim AS runtime

RUN apt-get update && apt-get install -y nginx gettext-base libnginx-mod-stream

ENV VIRTUAL_ENV=/app/.venv \
    PATH="/app/.venv/bin:$PATH"

COPY --from=builder ${VIRTUAL_ENV} ${VIRTUAL_ENV}

WORKDIR /app
COPY nginx.conf /etc/nginx/nginx.conf
COPY nginx.http.conf.template .
COPY static static

COPY start.sh /app/start.sh

RUN chmod +x /app/start.sh

ENTRYPOINT ["/app/start.sh"]
