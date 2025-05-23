#!/usr/bin/env bash
set -euo pipefail

# ------------------------------------------------------------------------------
# Description: Delete a Nova instance in Wandelbots staging.
# Usage:
#   1. Ensure the following environment variables are set:
#       PORTAL_PROD_INSTANCE_ID
#       PORTAL_PROD_ACCESS_TOKEN
#   2. Mark script as executable: chmod +x delete_nova_instance.sh
#   3. Run it: ./delete_nova_instance.sh
#
# Optional:
#   INSECURE_CURL="true" if you need to skip SSL verification for self-signed certs.
#
# Example:
#   export PORTAL_PROD_INSTANCE_ID="abc123"
#   export PORTAL_PROD_ACCESS_TOKEN="your-token"
#   ./delete_nova_instance.sh
# ------------------------------------------------------------------------------

: "${PORTAL_PROD_INSTANCE_ID:?Environment variable PORTAL_PROD_INSTANCE_ID is not set or empty.}"
: "${PORTAL_PROD_ACCESS_TOKEN:?Environment variable PORTAL_PROD_ACCESS_TOKEN is not set or empty.}"

echo "Deleting instance with ID: ${PORTAL_PROD_INSTANCE_ID}"

CURL_ARGS=("--fail" "--location")
if [ "${INSECURE_CURL:-}" = "true" ]; then
  CURL_ARGS+=("--insecure")
fi

curl -X DELETE \
  "https://io.wandelbots.io/instance/${PORTAL_PROD_INSTANCE_ID}" \
  -H 'accept: application/json' \
  -H "Authorization: Bearer ${PORTAL_PROD_ACCESS_TOKEN}" \
  "${CURL_ARGS[@]}"

echo "Instance ${PORTAL_PROD_INSTANCE_ID} has been deleted successfully."
