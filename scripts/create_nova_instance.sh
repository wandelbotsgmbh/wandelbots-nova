#!/usr/bin/env bash
set -euo pipefail

# ------------------------------------------------------------------------------
# Description: Retrieve access token, create instance, create default cell,
#              and verify that ProgramEngine is up.
# ------------------------------------------------------------------------------

# --- 1) CHECK REQUIRED ENV VARS ------------------------------------------------
: "${PORTAL_PROD_REFRESH_URL:?Missing PORTAL_PROD_REFRESH_URL}"
: "${PORTAL_PROD_REFRESH_CLIENT_ID:?Missing PORTAL_PROD_REFRESH_CLIENT_ID}"
: "${PORTAL_PROD_REFRESH_TOKEN:?Missing PORTAL_PROD_REFRESH_TOKEN}"

echo "## Updating the refresh token..."
echo "Refresh URL: ${PORTAL_PROD_REFRESH_URL}"
PORTAL_PROD_ACCESS_TOKEN="$(curl --request POST \
  --url "${PORTAL_PROD_REFRESH_URL}" \
  --header 'content-type: application/x-www-form-urlencoded' \
  --data grant_type=refresh_token \
  --data "client_id=${PORTAL_PROD_REFRESH_CLIENT_ID}" \
  --data "refresh_token=${PORTAL_PROD_REFRESH_TOKEN}" \
  | jq -r .access_token)"

if [ -z "$PORTAL_PROD_ACCESS_TOKEN" ] || [ "$PORTAL_PROD_ACCESS_TOKEN" = "null" ]; then
  echo "[ERROR] Failed to retrieve a valid access token."
  exit 1
fi

echo "Access-token acquired."

# --- 3) CREATE SANDBOX INSTANCE -----------------------------------------------
SANDBOX_NAME="svcmgr-${GITHUB_RUN_ID:-local-run}"
echo "Creating instance: ${SANDBOX_NAME}"

if ! INSTANCE_RESPONSE="$(curl -X "POST" "https://io.wandelbots.io/instance" \
  -H "accept: application/json" \
  -H "Authorization: Bearer ${PORTAL_PROD_ACCESS_TOKEN}" \
  -H "Content-Type: application/json" \
  -d "{\"sandbox_name\": \"${SANDBOX_NAME}\"}")"; then
  echo "Failed to create a new instance."
  echo "Response from create instance: ${INSTANCE_RESPONSE}"
  exit 13
fi

echo "Instance creation response: ${INSTANCE_RESPONSE}"

PORTAL_PROD_HOST="$(echo "${INSTANCE_RESPONSE}" | jq -r .host)"
PORTAL_PROD_INSTANCE_ID="$(echo "${INSTANCE_RESPONSE}" | jq -r .instance_id)"

[[ -z "${PORTAL_PROD_HOST}" || "${PORTAL_PROD_HOST}" == "null" ]] && {
  echo "[ERROR] No host returned"; exit 1; }
[[ -z "${PORTAL_PROD_INSTANCE_ID}" || "${PORTAL_PROD_INSTANCE_ID}" == "null" ]] && {
  echo "[ERROR] No instance-id returned"; exit 1; }

echo "Host: ${PORTAL_PROD_HOST}"
echo "Instance-ID: ${PORTAL_PROD_INSTANCE_ID}"

API_URL="https://${PORTAL_PROD_HOST}/api"

# --- 4) WAIT UNTIL /api/v2/cells RETURNS A NON-EMPTY ARRAY --------------------
CURL_ARGS=(--silent --show-error --fail-with-body --insecure)

echo "Waiting for cells to appear (timeout: 180 s)..."
START_TIME=$(date +%s)

while :; do
  # Capture body + HTTP code in one shot
  HTTP_AND_BODY="$(
    curl "${CURL_ARGS[@]}" \
         -H "Authorization: Bearer ${PORTAL_PROD_ACCESS_TOKEN}" \
         -H "Accept: application/json" \
         "${API_URL}/v2/cells" -w '\n%{http_code}' || true
  )"

  BODY="$(echo "${HTTP_AND_BODY}" | head -n -1)"
  HTTP_CODE="$(echo "${HTTP_AND_BODY}" | tail -n 1)"

  echo "DEBUG(cells): HTTP ${HTTP_CODE}"
  [[ -n "${BODY}" ]] && echo "DEBUG(cells) body: $(echo "${BODY}" | head -c 200)…"

  # Proceed only if we got 200 and valid JSON
  if [[ "${HTTP_CODE}" == "200" ]] && echo "${BODY}" | jq empty >/dev/null 2>&1; then
      COUNT="$(echo "${BODY}" | jq 'length')"
      echo "Current cell count: ${COUNT}"
      if (( COUNT > 0 )); then
          echo "✅ At least one cell present."
          break
      fi
  fi

  if (( $(date +%s) - START_TIME > 180 )); then
      echo "❌ Timeout: still no cells after 180 s."; exit 1
  fi
  sleep 5
done

# --- 5) WAIT FOR ProgramEngine INSIDE THE CELL ----------------------------------
echo "Waiting for ProgramEngine to reach state 'Running' (timeout: 120 s)…"
STATUS_URL="${API_URL}/v2/cells/cell/status"
START_TIME=$(date +%s)

while :; do
  HTTP_AND_BODY="$(curl "${CURL_ARGS[@]}" \
                        -H "Authorization: Bearer ${PORTAL_PROD_ACCESS_TOKEN}" \
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
           | jq -r '.service_status[]? | select(.service=="ProgramEngine") | .status.code')"

  echo "ProgamEngine: ${STATUS:-<empty>}"
  [[ "${STATUS}" == "Running" ]] && break

  if (( $(date +%s) - START_TIME > 120 )); then
    echo "❌ Timeout: ProgramEngine did not reach 'Running'"; exit 1
  fi
  sleep 10
done
echo "✅ Cell ready – ProgramEngine is Running."

# --- 7) EXPORT VARS FOR DOWNSTREAM STEPS -------------------------------------
export PORTAL_PROD_ACCESS_TOKEN
export PORTAL_PROD_HOST
export PORTAL_PROD_INSTANCE_ID
echo "NOVA instance and cell created successfully."
