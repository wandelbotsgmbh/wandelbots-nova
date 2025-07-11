#!/bin/bash

# Nova Robot Control Extension (WebSocket) - Installation Script
# This script packages and installs the VS Code extension with WebSocket support

set -e

echo "🚀 Installing Nova Robot Control VS Code Extension (WebSocket)..."
echo "📡 Features: Real-time robot control via WebSocket connection"
echo

# Check if we're in the right directory
if [ ! -f "package.json" ]; then
    echo "❌ Error: Must run from vscode-extension directory"
    echo "   Usage: cd vscode-extension && ./install.sh"
    exit 1
fi

# Check if vsce is installed
if ! command -v vsce &> /dev/null; then
    echo "📦 Installing VS Code Extension Manager (vsce)..."
    npm install -g vsce
    echo "✅ vsce installed"
fi

# Package the extension
echo "📦 Packaging WebSocket extension..."
vsce package

# Find the generated VSIX file
VSIX_FILE=$(ls nova-robot-control-*.vsix | head -n 1)

if [ -z "$VSIX_FILE" ]; then
    echo "❌ Error: Failed to create VSIX package"
    exit 1
fi

echo "✅ Extension packaged: $VSIX_FILE"

# Install the extension
echo "📥 Installing extension in VS Code..."

# Try different VS Code command names
if command -v code &> /dev/null; then
    code --install-extension "$VSIX_FILE"
elif command -v code-insiders &> /dev/null; then
    code-insiders --install-extension "$VSIX_FILE"
else
    echo "⚠️  Could not find 'code' command."
    echo "   Please install manually:"
    echo "   1. Open VS Code"
    echo "   2. Press Ctrl+Shift+P (Cmd+Shift+P on Mac)"
    echo "   3. Type 'Extensions: Install from VSIX'"
    echo "   4. Select: $(pwd)/$VSIX_FILE"
    exit 0
fi

echo "✅ Extension installed successfully!"
echo
echo "🎉 Nova Robot Control is now available in VS Code!"
echo
echo "Next steps:"
echo "1. Open a workspace with a Nova Python project"
echo "2. Run a Nova program with robots"  
echo "3. Look for the robot icon 🤖 in the status bar"
echo "4. Click it to control your robots!"
echo
echo "Commands available in Command Palette (Ctrl+Shift+P):"
echo "- Nova: Show Robot Controls"
echo "- Nova: Detect Robots" 
echo "- Nova: Refresh Robot List"
echo
