#!/usr/bin/env bash
set -euo pipefail

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

CURL_ARGS=("--fail" "--location")
if [ "${INSECURE_CURL:-}" = "true" ]; then
  CURL_ARGS+=("--insecure")
fi

echo "Fetching all instances..."

# Get all instances
INSTANCES=$(curl --request GET \
  --url "https://api.portal.wandelbots.io/v1/instances" \
  --header 'accept: application/json' \
  --header "Authorization: Bearer ${PORTAL_PROD_ACCESS_TOKEN}" \
  "${CURL_ARGS[@]}")

# Extract instance IDs using jq
INSTANCE_IDS=$(echo "${INSTANCES}" | jq -r '.instances[].instance_id')

if [ -z "${INSTANCE_IDS}" ]; then
  echo "No instances found."
  exit 0
fi

echo "Found instances. Proceeding with deletion..."

# Delete each instance
for INSTANCE_ID in ${INSTANCE_IDS}; do
  echo "Deleting instance with ID: ${INSTANCE_ID}"

  curl --request DELETE \
    --url "https://api.portal.wandelbots.io/v1/instances/${INSTANCE_ID}" \
    --header 'accept: application/json' \
    --header "Authorization: Bearer ${PORTAL_PROD_ACCESS_TOKEN}" \
    "${CURL_ARGS[@]}"

  echo "Instance ${INSTANCE_ID} has been deleted successfully."
done

echo "All instances have been deleted."
