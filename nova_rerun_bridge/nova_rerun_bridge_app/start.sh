#!/bin/sh

# Substitute environment variables in the NGINX config template
envsubst '${BASE_PATH}' < /app/nginx.http.conf.template > /etc/nginx/conf.d/default.conf

# Start NGINX
nginx &

# Start the Python processes
python /app/nova_rerun_bridge/polling/populate.py &

# Wait for all background processes to finish
wait