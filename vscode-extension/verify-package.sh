#!/bin/bash

# Nova Robot Control Extension - Verification Script
# This script verifies the extension package contents

EXTENSION_FILE="nova-robot-control-websocket-2.0.0.vsix"
EXTENSION_DIR="/Users/stefanwagner/Git/wandelbots-nova/vscode-extension"

echo "🔍 Nova Robot Control Extension - Verification"
echo "=============================================="

cd "$EXTENSION_DIR"

if [ ! -f "$EXTENSION_FILE" ]; then
    echo "❌ Extension file not found: $EXTENSION_FILE"
    exit 1
fi

echo "✅ Extension file found: $EXTENSION_FILE"
echo ""

# Show package information
echo "📦 Package Information:"
echo "========================"
vsce ls --tree "$EXTENSION_FILE" | head -20

echo ""
echo "📄 Package Details:"
echo "==================="
stat -f "Size: %z bytes" "$EXTENSION_FILE"
echo "Created: $(stat -f %Sm $EXTENSION_FILE)"

echo ""
echo "🚀 Ready to Install!"
echo "Run: ./install-websocket.sh"
