#!/bin/bash

# Script to install Wandelbots Viewer extension directly to code-server extensions folder
# This method bypasses the need for `code --install-extension`

set -e

echo "Installing Wandelbots Viewer extension directly..."

# Extension paths
EXTENSION_VSIX="./wandelbots-nova-0.0.1.vsix"
EXTENSION_NAME="wandelbots.wandelbots-nova-0.0.1"
EXTENSIONS_DIR="/config/.local/share/code-server/extensions"
INSTALL_DIR="${EXTENSIONS_DIR}/${EXTENSION_NAME}"

# Check if VSIX file exists
if [ ! -f "$EXTENSION_VSIX" ]; then
    echo "Error: Extension VSIX file not found at $EXTENSION_VSIX"
    echo "Please make sure the extension is packaged first with 'vsce package'"
    exit 1
fi

# Create extensions directory if it doesn't exist
mkdir -p "$EXTENSIONS_DIR"

# Remove existing installation if it exists
if [ -d "$INSTALL_DIR" ]; then
    echo "Removing existing installation..."
    rm -rf "$INSTALL_DIR"
fi

# Create installation directory
mkdir -p "$INSTALL_DIR"

# Extract VSIX file (VSIX is just a ZIP file)
echo "Extracting extension..."
cd "$INSTALL_DIR"

# Use unzip to extract the VSIX
unzip -q "$EXTENSION_VSIX"

# Move extension files from the 'extension' subfolder to the root
if [ -d "extension" ]; then
    mv extension/* .
    rmdir extension
fi

# Remove unnecessary files
rm -f "[Content_Types].xml" "extension.vsixmanifest"

# Set proper ownership
chown -R abc:abc "$INSTALL_DIR"

echo "Extension installed successfully to: $INSTALL_DIR"
echo ""
echo "Extension contents:"
ls -la "$INSTALL_DIR"
echo ""
echo "The extension will be available when code-server restarts."
echo "You can restart code-server or reload the VS Code window to activate it."
