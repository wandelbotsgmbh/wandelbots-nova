#!/usr/bin/env bash
# Setup a Nova instance for the dual-arm policy examples.
#
# Imports the cell configuration (two UR5e arms with mounting + gripper TCP)
# from cell-setup.tar.gz using the Nova REST API.
#
# Usage:
#   ./setup_cell.sh                          # uses NOVA_API env var
#   ./setup_cell.sh http://172.31.11.129     # explicit host
#
# The cell export contains:
#   - ur5e-left:  mounted at [0, 245, 0] mm, rotateX(-135°) → rotateZ(90°)
#   - ur5e-right: mounted at [0, -245, 0] mm, rotateX(135°) → rotateZ(90°)
#   - Both have a "gripper" TCP at [0, -60.05, 1.7] mm, rz=90°

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BACKUP_FILE="$SCRIPT_DIR/cell-setup.tar.gz"

HOST="${1:-${NOVA_API:-}}"
if [ -z "$HOST" ]; then
    echo "Usage: $0 <nova-host>  (e.g. http://172.31.11.129)"
    echo "   or: NOVA_API=http://172.31.11.129 $0"
    exit 1
fi

HOST="${HOST%/}"
API="$HOST/api/v2"

restore() {
    echo "Restoring cell configuration (pass $1)..."
    curl -sS --max-time 300 -X POST "$API/system/configuration" \
        -H "Content-Type: application/gzip" \
        --data-binary "@$BACKUP_FILE" -o /dev/null 2>/dev/null &
    CURL_PID=$!
    while kill -0 $CURL_PID 2>/dev/null; do printf "."; sleep 3; done
    wait $CURL_PID 2>/dev/null || true
    echo ""
}

wait_for_api() {
    echo "Waiting for instance..."
    for i in $(seq 1 40); do
        if curl -fsS --max-time 5 "$API/cells/cell/controllers" >/dev/null 2>&1; then
            return 0
        fi
        sleep 5
    done
    echo "ERROR: instance not reachable at $HOST after 200s"
    exit 1
}

verify() {
    OK=true
    for ctrl in ur5e-left ur5e-right; do
        TCPS=$(curl -fsS "$API/virtual-controllers/$ctrl/motion-groups/0@$ctrl/tcps" 2>/dev/null || echo '[]')
        HAS_GRIPPER=$(echo "$TCPS" | python3 -c "import json,sys; print(any(t['id']=='gripper' for t in json.load(sys.stdin)))" 2>/dev/null)
        if [ "$HAS_GRIPPER" != "True" ]; then
            OK=false
        fi
    done
    echo "$OK"
}

# Pass 1: creates controllers (mounting/TCP may not apply yet)
restore 1
wait_for_api

if [ "$(verify)" = "true" ]; then
    echo "✓ Cell setup complete on first pass."
else
    # Pass 2: instance is up, controllers exist — mounting/TCP now applies
    # The instance may restart after this restore.
    echo "Applying mounting + TCP configuration..."
    restore 2
    sleep 10
    wait_for_api
fi

# Final verification
echo ""
echo "Verifying..."
sleep 10
CONTROLLERS=$(curl -fsS "$API/cells/cell/controllers")
echo "Controllers: $CONTROLLERS"

for ctrl in ur5e-left ur5e-right; do
    POS=$(curl -fsS "$API/virtual-controllers/$ctrl/motion-groups/0@$ctrl/mounting" 2>/dev/null \
        | python3 -c "import json,sys; print(json.load(sys.stdin).get('position','?'))" 2>/dev/null)
    TCPS=$(curl -fsS "$API/virtual-controllers/$ctrl/motion-groups/0@$ctrl/tcps" 2>/dev/null \
        | python3 -c "import json,sys; [print(f'    {t[\"id\"]}: pos={t[\"position\"]}') for t in json.load(sys.stdin)]" 2>/dev/null)
    echo "  $ctrl: mounting=$POS"
    echo "$TCPS"
done

if [ "$(verify)" = "true" ]; then
    echo ""
    echo "✓ Cell setup complete. Ready for dual-arm examples."
else
    echo ""
    echo "⚠ Setup incomplete. Check the instance manually."
    exit 1
fi
