#!/bin/sh

# Substitute environment variables in the NGINX config template
envsubst '${BASE_PATH}' < /app/nginx.http.conf.template > /etc/nginx/conf.d/default.conf

# Start NGINX
nginx &

python -m rerun --serve-web --web-viewer-port 3000 --hide-welcome-screen --expect-data-soon &

# Wait for all background processes to finish
wait