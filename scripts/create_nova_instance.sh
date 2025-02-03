#!/usr/bin/env bash
set -euo pipefail

# ------------------------------------------------------------------------------
# Description: Retrieve access token, create instance, and check service availability.
# Usage:
#   1. Ensure the following environment variables are set (see "Required env vars").
#   2. Mark script as executable: chmod +x create_instance_check_connection.sh
#   3. Ensure the following environment variables are set via `source .env`:
#         PORTAL_STG_INSTANCE_ID
#         PORTAL_STG_ACCESS_TOKEN
#         PORTAL_STG_HOST
#   3. Run it: ./create_instance_check_connection.sh
#
# Required env vars (you can pass them from your CI Secrets/Env):
#   PORTAL_STG_REFRESH_URL          The refresh token URL to obtain access token
#   PORTAL_STG_REFRESH_CLIENT_ID    The refresh token client ID
#   PORTAL_STG_REFRESH_TOKEN        The refresh token value
#   PROJECT_VERSION                 (Optional) Your project/version label
#   GITHUB_RUN_ID                   (Optional) Unique run ID (used in sandbox name)
#   API_VERSION                     e.g. "v1"
#
# Optional:
#   INSECURE_CURL                   Set to "true" if you want to skip SSL checks (for self-signed)
#
# Outputs (exported):
#   PORTAL_STG_ACCESS_TOKEN         Access token from the refresh endpoint
#   PORTAL_STG_HOST                Host of the newly created instance
#   PORTAL_STG_INSTANCE_ID         Instance ID of the newly created instance
# ------------------------------------------------------------------------------

# --- 1) CHECK REQUIRED ENV VARS ---

: "${PORTAL_STG_REFRESH_URL:?Environment variable PORTAL_STG_REFRESH_URL is not set or empty.}"
: "${PORTAL_STG_REFRESH_CLIENT_ID:?Environment variable PORTAL_STG_REFRESH_CLIENT_ID is not set or empty.}"
: "${PORTAL_STG_REFRESH_TOKEN:?Environment variable PORTAL_STG_REFRESH_TOKEN is not set or empty.}"
: "${API_VERSION:?Environment variable API_VERSION is not set or empty.}"

# Some variables might be optional. If they are used below, uncomment and ensure they're set:
#: "${PROJECT_VERSION:?Environment variable PROJECT_VERSION is not set or empty.}"
#: "${GITHUB_RUN_ID:?Environment variable GITHUB_RUN_ID is not set or empty.}"

# --- 2) RETRIEVE ACCESS TOKEN ---
echo "## Updating the refresh token..."
PORTAL_STG_ACCESS_TOKEN="$(curl --request POST \
  --url "${PORTAL_STG_REFRESH_URL}" \
  --header 'content-type: application/x-www-form-urlencoded' \
  --data grant_type=refresh_token \
  --data "client_id=${PORTAL_STG_REFRESH_CLIENT_ID}" \
  --data "refresh_token=${PORTAL_STG_REFRESH_TOKEN}" \
  | jq -r .access_token)"

if [ -z "$PORTAL_STG_ACCESS_TOKEN" ] || [ "$PORTAL_STG_ACCESS_TOKEN" = "null" ]; then
  echo "[ERROR] Failed to retrieve a valid access token."
  exit 1
fi

echo "Access token received."

# --- 3) CREATE INSTANCE ---
# If PROJECT_VERSION or GITHUB_RUN_ID are not used in your naming, simplify "sandbox_name" as needed.
SANDBOX_NAME="svcmgr-${GITHUB_RUN_ID:-local-run}"
echo "Creating instance with sandbox name: ${SANDBOX_NAME}"

INSTANCE_RESPONSE="$(curl --fail -X "POST" "https://io.stg.wandelbots.io/instance" \
  -H "accept: application/json" \
  -H "Authorization: Bearer ${PORTAL_STG_ACCESS_TOKEN}" \
  -H "Content-Type: application/json" \
  -d "{\"sandbox_name\": \"${SANDBOX_NAME}\"}")"

echo "Response from create instance: ${INSTANCE_RESPONSE}"
PORTAL_STG_HOST="$(echo "${INSTANCE_RESPONSE}" | jq -r .host)"
PORTAL_STG_INSTANCE_ID="$(echo "${INSTANCE_RESPONSE}" | jq -r .instance_id)"

if [ -z "$PORTAL_STG_HOST" ] || [ "$PORTAL_STG_HOST" = "null" ]; then
  echo "[ERROR] Failed to retrieve a valid host from the instance creation response."
  exit 1
fi

if [ -z "$PORTAL_STG_INSTANCE_ID" ] || [ "$PORTAL_STG_INSTANCE_ID" = "null" ]; then
  echo "[ERROR] Failed to retrieve a valid instance ID from the instance creation response."
  exit 1
fi

echo "Host: ${PORTAL_STG_HOST}"
echo "Instance ID: ${PORTAL_STG_INSTANCE_ID}"

# --- 4) CHECK SERVICE AVAILABILITY ---
# By default, use secure curl. If you have a self-signed certificate and want to skip verification,
# set INSECURE_CURL="true".
CURL_ARGS=("--fail" "--location" "--retry-all-errors" "--retry" "7" "--retry-max-time" "200")
if [ "${INSECURE_CURL:-}" = "true" ]; then
  CURL_ARGS+=("--insecure")
fi

API_URL="https://${PORTAL_STG_HOST}/api/${API_VERSION}"
echo "Checking service availability at: ${API_URL}/cells"

# Make the request
curl "${CURL_ARGS[@]}" \
  --header "Authorization: Bearer ${PORTAL_STG_ACCESS_TOKEN}" \
  "${API_URL}/cells"

# If we reach here, the service is accessible.
echo "Service is up and reachable."

# Export environment variables for further steps if needed:
export PORTAL_STG_ACCESS_TOKEN
export PORTAL_STG_HOST
export PORTAL_STG_INSTANCE_ID
echo "NOVA instance created successfully"
