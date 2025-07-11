#!/bin/bash

# Nova Robot Control Extension - Installation Script
# This script installs the VS Code extension

EXTENSION_FILE="nova-robot-control-websocket-2.0.0.vsix"
EXTENSION_DIR="/Users/stefanwagner/Git/wandelbots-nova/vscode-extension"

echo "🤖 Nova Robot Control Extension - Installation"
echo "=============================================="

# Check if VS Code is installed
if ! command -v code &> /dev/null; then
    echo "❌ VS Code CLI not found. Please install VS Code or add it to your PATH."
    echo "   To add VS Code to PATH:"
    echo "   1. Open VS Code"
    echo "   2. Press Cmd+Shift+P"
    echo "   3. Type 'shell command' and select 'Install code command in PATH'"
    exit 1
fi

echo "✅ VS Code CLI found"

# Check if extension file exists
if [ ! -f "$EXTENSION_DIR/$EXTENSION_FILE" ]; then
    echo "❌ Extension file not found: $EXTENSION_FILE"
    echo "   Please build the extension first using: vsce package"
    exit 1
fi

echo "✅ Extension file found: $EXTENSION_FILE"

# Install the extension
echo "📦 Installing extension..."
cd "$EXTENSION_DIR"
code --install-extension "$EXTENSION_FILE" --force

if [ $? -eq 0 ]; then
    echo "✅ Extension installed successfully!"
    echo ""
    echo "🚀 Getting Started:"
    echo "1. Start a Nova program with robots"
    echo "2. Click the Nova status bar item in VS Code"
    echo "3. Or run command: 'Nova: Show Robot Control Panel'"
    echo ""
    echo "⚙️  Configuration:"
    echo "• Go to VS Code Settings → Extensions → Nova Robot Control"
    echo "• Configure host/port for remote connections"
    echo ""
    echo "📖 Documentation: README-websocket.md"
else
    echo "❌ Failed to install extension"
    exit 1
fi
