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
: "${API_VERSION:?Missing API_VERSION}"       # e.g. "v2"

# --- 2) RETRIEVE ACCESS TOKEN --------------------------------------------------
echo "## Refreshing access token..."
PORTAL_STG_ACCESS_TOKEN="$(
  curl --request POST \
       --url  "${PORTAL_STG_REFRESH_URL}" \
       --header 'content-type: application/x-www-form-urlencoded' \
       --data grant_type=refresh_token \
       --data "client_id=${PORTAL_STG_REFRESH_CLIENT_ID}" \
       --data "refresh_token=${PORTAL_STG_REFRESH_TOKEN}" \
  | jq -r .access_token
)"

if [[ -z "${PORTAL_STG_ACCESS_TOKEN}" || "${PORTAL_STG_ACCESS_TOKEN}" == "null" ]]; then
  echo "[ERROR] Failed to retrieve a valid access token"; exit 1
fi
echo "Access-token acquired."

# --- 3) CREATE SANDBOX INSTANCE -----------------------------------------------
SANDBOX_NAME="svcmgr-${GITHUB_RUN_ID:-local-run}"
echo "Creating instance: ${SANDBOX_NAME}"

INSTANCE_RESPONSE="$(
  curl -sS -X POST "https://io.stg.wandelbots.io/instance" \
       -H "accept: application/json" \
       -H "Authorization: Bearer ${PORTAL_STG_ACCESS_TOKEN}" \
       -H "Content-Type: application/json" \
       -d "{\"sandbox_name\":\"${SANDBOX_NAME}\"}"
)"

PORTAL_STG_HOST="$(echo "${INSTANCE_RESPONSE}" | jq -r .host)"
PORTAL_STG_INSTANCE_ID="$(echo "${INSTANCE_RESPONSE}" | jq -r .instance_id)"

[[ -z "${PORTAL_STG_HOST}"        || "${PORTAL_STG_HOST}"        == "null" ]] && {
  echo "[ERROR] No host returned"; exit 1; }
[[ -z "${PORTAL_STG_INSTANCE_ID}" || "${PORTAL_STG_INSTANCE_ID}" == "null" ]] && {
  echo "[ERROR] No instance-id returned"; exit 1; }

echo "Host: ${PORTAL_STG_HOST}"
echo "Instance-ID: ${PORTAL_STG_INSTANCE_ID}"

# --- 4) WAIT UNTIL /cells RETURNS VALID JSON ----------------------------------
CURL_ARGS=(--fail --location --retry-all-errors --retry 30 --retry-max-time 200)
[[ "${INSECURE_CURL:-}" == "true" ]] && CURL_ARGS+=(--insecure)

API_URL="https://${PORTAL_STG_HOST}/api/${API_VERSION}"

echo "Waiting for service to answer at ${API_URL}/cells ..."
for (( i=1; i<=5; i++ )); do
  echo "  • Attempt $i/5"
  RESPONSE="$(curl -sS "${CURL_ARGS[@]}" \
                    -H "Authorization: Bearer ${PORTAL_STG_ACCESS_TOKEN}" \
                    -H "Accept: application/json" \
                    "${API_URL}/cells/cell/status" || true)"
  if [[ -n "${RESPONSE}" && $(echo "${RESPONSE}" | jq empty >/dev/null 2>&1; echo $?) -eq 0 ]]; then
    echo "✅ API responded with valid JSON"; break
  fi
  [[ $i -eq 5 ]] && { echo "❌ API never responded with JSON"; exit 1; }
  sleep 5
done

# --- 5) CREATE THE DEFAULT CELL ----------------------------------------------
echo "Creating cell 'cell' ..."
curl -sS -X PUT "${API_URL}/internal/cells/cell" \
     -H "Authorization: Bearer ${PORTAL_STG_ACCESS_TOKEN}" \
     -H "Content-Type: application/json" \
     -H "Accept: application/json" \
     -d '{"name":"cell"}' | jq .

# --- 6) WAIT FOR ROBOTENGINE INSIDE THE CELL ----------------------------------
echo "Waiting for RobotEngine to reach state 'Running' (timeout: 120 s)..."
START_TIME=$(date +%s)
while :; do
  STATUS="$(curl -sS "${API_URL}/cells/cell/status" \
                -H "Authorization: Bearer ${PORTAL_STG_ACCESS_TOKEN}" \
                -H "Accept: application/json" \
          | jq -r '.service_status[] | select(.service=="RobotEngine") | .status.code')"

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
