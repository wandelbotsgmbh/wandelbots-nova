#!/usr/bin/env bash
set -euo pipefail

# ------------------------------------------------------------------------------
# Description: Retrieve access token, create instance, create default cell,
#              and verify that RobotEngine is up.
# ------------------------------------------------------------------------------

# --- 1) CHECK REQUIRED ENV VARS ------------------------------------------------
: "${PORTAL_STG_REFRESH_URL:?Missing PORTAL_STG_REFRESH_URL}"
: "${PORTAL_STG_REFRESH_CLIENT_ID:?Missing PORTAL_STG_REFRESH_CLIENT_ID}"
: "${PORTAL_STG_REFRESH_TOKEN:?Missing PORTAL_STG_REFRESH_TOKEN}"

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

echo "Access-token acquired."

# --- 3) CREATE SANDBOX INSTANCE -----------------------------------------------
SANDBOX_NAME="svcmgr-${GITHUB_RUN_ID:-local-run}"
echo "Creating instance: ${SANDBOX_NAME}"

if ! INSTANCE_RESPONSE="$(curl -X "POST" "https://io.stg.wandelbots.io/instance" \
  -H "accept: application/json" \
  -H "Authorization: Bearer ${PORTAL_STG_ACCESS_TOKEN}" \
  -H "Content-Type: application/json" \
  -d "{\"sandbox_name\": \"${SANDBOX_NAME}\"}")"; then
  echo "Failed to create a new instance."
  echo "Response from create instance: ${INSTANCE_RESPONSE}"
  exit 1
fi

echo "Instance creation response: ${INSTANCE_RESPONSE}"

PORTAL_STG_HOST="$(echo "${INSTANCE_RESPONSE}" | jq -r .host)"
PORTAL_STG_INSTANCE_ID="$(echo "${INSTANCE_RESPONSE}" | jq -r .instance_id)"

[[ -z "${PORTAL_STG_HOST}" || "${PORTAL_STG_HOST}" == "null" ]] && {
  echo "[ERROR] No host returned"; exit 1; }
[[ -z "${PORTAL_STG_INSTANCE_ID}" || "${PORTAL_STG_INSTANCE_ID}" == "null" ]] && {
  echo "[ERROR] No instance-id returned"; exit 1; }

echo "Host: ${PORTAL_STG_HOST}"
echo "Instance-ID: ${PORTAL_STG_INSTANCE_ID}"

API_URL="https://${PORTAL_STG_HOST}/api"

# --- 4) CREATE THE DEFAULT CELL ----------------------------------------------
CURL_ARGS=(--silent --show-error --fail-with-body --insecure)

echo "Creating cell 'cell' ..."
HTTP_AND_BODY="$(curl "${CURL_ARGS[@]}" --request PUT \
                      --url "${API_URL}/v1/cells?completionTimeout=180" \
                      --header "Authorization: Bearer ${PORTAL_STG_ACCESS_TOKEN}" \
                      --header "Content-Type: application/json" \
                      --header "Accept: application/json" \
                      --data '{"name": "cell"}' -w '\n%{http_code}')"

BODY="$(echo "${HTTP_AND_BODY}" | head -n -1)"
HTTP_CODE="$(echo "${HTTP_AND_BODY}" | tail -n 1)"

echo "DEBUG(create): HTTP ${HTTP_CODE}"
[[ -n "${BODY}" ]] && echo "DEBUG(create) body: $(echo "${BODY}" | head -c 200)…"

[[ "${HTTP_CODE}" != "201" && "${HTTP_CODE}" != "200" ]] && {
  echo "❌ Failed to create cell (HTTP ${HTTP_CODE})"; exit 1; }

echo "${BODY}" | jq .

# --- 5) WAIT FOR ROBOTENGINE INSIDE THE CELL ----------------------------------
echo "Waiting for RobotEngine to reach state 'Running' (timeout: 120 s)…"
STATUS_URL="${API_URL}/v2/cells/cell/status"
START_TIME=$(date +%s)

while :; do
  HTTP_AND_BODY="$(curl "${CURL_ARGS[@]}" \
                        -H "Authorization: Bearer ${PORTAL_STG_ACCESS_TOKEN}" \
                        -H "Accept: application/json" \
                        "${STATUS_URL}" -w '\n%{http_code}' || true)"

  BODY="$(echo "${HTTP_AND_BODY}" | head -n -1)"
  HTTP_CODE="$(echo "${HTTP_AND_BODY}" | tail -n 1)"

  echo "DEBUG(status): HTTP ${HTTP_CODE}"
  [[ -n "${BODY}" ]] && echo "DEBUG(status) body: $(echo "${BODY}" | head -c 200)…"

  # Retry on non-200 or non-JSON
  if [[ "${HTTP_CODE}" != "200" ]] || ! echo "${BODY}" | jq empty >/dev/null 2>&1; then
    echo "⚠️  Bad response; retrying in 10 s…"
    sleep 10; continue
  fi

  STATUS="$(echo "${BODY}" \
           | jq -r '.service_status[]? | select(.service=="RobotEngine") | .status.code')"

  echo "RobotEngine: ${STATUS:-<empty>}"
  [[ "${STATUS}" == "Running" ]] && break

  if (( $(date +%s) - START_TIME > 120 )); then
    echo "❌ Timeout: RobotEngine did not reach 'Running'"; exit 1
  fi
  sleep 10
done
echo "✅ Cell ready – RobotEngine is Running."

# --- 7) EXPORT VARS FOR DOWNSTREAM STEPS -------------------------------------
export PORTAL_STG_ACCESS_TOKEN
export PORTAL_STG_HOST
export PORTAL_STG_INSTANCE_ID
echo "NOVA instance and cell created successfully."
