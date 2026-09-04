#!/usr/bin/env bash
# Setup a Nova instance for the policy examples.
#
# Imports a cell configuration through the Nova REST API. Two cells ship here:
#
#   dual-arm (default)  two UR5e arms, for the dual-arm policy examples
#     - ur5e-left:  mounted at [0, 245, 0] mm, rotateX(-135°) → rotateZ(90°)
#     - ur5e-right: mounted at [0, -245, 0] mm, rotateX(135°) → rotateZ(90°)
#     - both carry a "gripper" TCP at [0, -60.05, 1.7] mm, rz=90°
#
#   umi                 one UR10e with the UMI gripper, for the LeRobot
#                       examples and pick_and_place_umi_ur10e.py
#     - ur10e mounted at the origin
#     - "umi_corrected" TCP at [-7.19, 0, 221.7] mm — the frame the choreo3
#       demonstrations were recorded in, NOT the cell's Flange. Fed the wrong
#       frame a policy does not fail, it stalls in a hover.
#
# Usage:
#   ./setup_cell.sh                              # dual-arm, NOVA_API env var
#   ./setup_cell.sh http://172.31.11.129         # dual-arm, explicit host
#   ./setup_cell.sh http://172.31.12.5 umi       # UMI gripper cell

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

HOST="${1:-${NOVA_API:-}}"
CELL_KIND="${2:-dual-arm}"

case "$CELL_KIND" in
    dual-arm)
        BACKUP_FILE="$SCRIPT_DIR/cell-setup.tar.gz"
        CONTROLLERS="ur5e-left ur5e-right"
        EXPECTED_TCP="gripper"
        ;;
    umi)
        BACKUP_FILE="$SCRIPT_DIR/umi-gripper-cell-setup.tar.gz"
        CONTROLLERS="ur10e"
        EXPECTED_TCP="umi_corrected"
        ;;
    *)
        echo "Unknown cell '$CELL_KIND'. Use 'dual-arm' or 'umi'."
        exit 1
        ;;
esac

if [ -z "$HOST" ]; then
    echo "Usage: $0 <nova-host> [dual-arm|umi]  (e.g. http://172.31.11.129)"
    echo "   or: NOVA_API=http://172.31.11.129 $0"
    exit 1
fi

HOST="${HOST%/}"
API="$HOST/api/v2"

restore() {
    echo "Restoring $CELL_KIND cell configuration (pass $1)..."
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
    for ctrl in $CONTROLLERS; do
        TCPS=$(curl -fsS "$API/cells/cell/virtual-controllers/$ctrl/motion-groups/0@$ctrl/tcps" 2>/dev/null || echo '[]')
        HAS_TCP=$(echo "$TCPS" | TCP="$EXPECTED_TCP" python3 -c "import json,os,sys; print(any(t['id']==os.environ['TCP'] for t in json.load(sys.stdin)))" 2>/dev/null)
        if [ "$HAS_TCP" != "True" ]; then
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

for ctrl in $CONTROLLERS; do
    POS=$(curl -fsS "$API/cells/cell/virtual-controllers/$ctrl/motion-groups/0@$ctrl/mounting" 2>/dev/null \
        | python3 -c "import json,sys; print(json.load(sys.stdin).get('position','?'))" 2>/dev/null)
    TCPS=$(curl -fsS "$API/cells/cell/virtual-controllers/$ctrl/motion-groups/0@$ctrl/tcps" 2>/dev/null \
        | python3 -c "import json,sys; [print(f'    {t[\"id\"]}: pos={t[\"position\"]}') for t in json.load(sys.stdin)]" 2>/dev/null)
    echo "  $ctrl: mounting=$POS"
    echo "$TCPS"
done

if [ "$(verify)" = "true" ]; then
    echo ""
    echo "✓ Cell setup complete ($CELL_KIND). Ready for the examples."
else
    echo ""
    echo "⚠ Setup incomplete. Check the instance manually."
    exit 1
fi
