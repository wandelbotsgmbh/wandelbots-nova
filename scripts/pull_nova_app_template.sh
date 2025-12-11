#!/bin/bash

# Script to test NOVA CLI against the SDK
# This script installs the NOVA CLI, creates a test app, and checks for differences

set -euo pipefail

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Configuration
TEST_APP_NAME="your-nova-app"
EXAMPLES_DIR="examples"
TEMP_DIR=$(mktemp -d)
CLI_VERSION="latest"
FORCE_OVERRIDE=false

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -f|--force)
            FORCE_OVERRIDE=true
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [OPTIONS]"
            echo "Options:"
            echo "  -f, --force    Force override existing test app directory"
            echo "  -h, --help     Show this help message"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use -h or --help for usage information"
            exit 1
            ;;
    esac
done

echo -e "${YELLOW}🚀 Starting NOVA CLI SDK compatibility test${NC}"

# Function to cleanup on exit
cleanup() {
    echo -e "${YELLOW}🧹 Cleaning up temporary files...${NC}"
    rm -rf "$TEMP_DIR"
    # Remove the test app if it exists and we're not in force mode
    # (in force mode, we want to keep the overridden app)
    if [ -d "$EXAMPLES_DIR/$TEST_APP_NAME" ] && [ "$FORCE_OVERRIDE" = false ]; then
        rm -rf "$EXAMPLES_DIR/$TEST_APP_NAME"
    fi
}
trap cleanup EXIT

# Check if we're in the right directory
if [ ! -f "pyproject.toml" ] || [ ! -d "nova" ]; then
    echo -e "${RED}❌ Error: This script must be run from the root of the wandelbots-nova repository${NC}"
    exit 1
fi

echo -e "${YELLOW}📦 Installing NOVA CLI...${NC}"

# Install NOVA CLI using homebrew (for macOS/Linux)
if command -v brew &> /dev/null; then
    echo "Installing NOVA CLI via homebrew..."
    brew install wandelbotsgmbh/wandelbots/nova
else
    echo -e "${YELLOW}⚠️  Homebrew not found, attempting manual installation...${NC}"

    # Detect OS and architecture
    OS=$(uname -s | tr '[:upper:]' '[:lower:]')
    ARCH=$(uname -m)

    case $ARCH in
        x86_64)
            ARCH="amd64"
            ;;
        arm64|aarch64)
            ARCH="arm64"
            ;;
        *)
            echo -e "${RED}❌ Unsupported architecture: $ARCH${NC}"
            exit 1
            ;;
    esac

    # Download NOVA CLI
    DOWNLOAD_URL="https://github.com/wandelbots/nova-cli/releases/latest/download/nova-${OS}-${ARCH}"
    echo "Downloading NOVA CLI from: $DOWNLOAD_URL"

    if ! curl -L -o "$TEMP_DIR/nova" "$DOWNLOAD_URL"; then
        echo -e "${RED}❌ Failed to download NOVA CLI${NC}"
        exit 1
    fi

    chmod +x "$TEMP_DIR/nova"
    sudo mv "$TEMP_DIR/nova" /usr/local/bin/nova

    # Verify installation
    if ! command -v nova &> /dev/null; then
        echo -e "${RED}❌ NOVA CLI installation failed${NC}"
        exit 1
    fi
fi

# Verify CLI installation
echo -e "${GREEN}✅ NOVA CLI installed successfully${NC}"
nova version

# Create backup of examples directory
echo -e "${YELLOW}📋 Creating backup of examples directory...${NC}"
cp -r "$EXAMPLES_DIR" "$TEMP_DIR/examples_backup"

# Check if test app already exists
if [ -d "$EXAMPLES_DIR/$TEST_APP_NAME" ]; then
    if [ "$FORCE_OVERRIDE" = true ]; then
        echo -e "${YELLOW}⚠️  Test app directory already exists. Force override enabled, removing existing directory...${NC}"
        rm -rf "$EXAMPLES_DIR/$TEST_APP_NAME"
    else
        echo -e "${YELLOW}⚠️  Test app directory already exists. Creating temporary copy for comparison...${NC}"
        # Create a backup of the existing directory for comparison
        cp -r "$EXAMPLES_DIR/$TEST_APP_NAME" "$TEMP_DIR/existing_app_backup"
    fi
fi

# Create test app using NOVA CLI
echo -e "${YELLOW}🔧 Creating test app with NOVA CLI...${NC}"

# Change to examples directory to create the app there
cd "$EXAMPLES_DIR"

# Create the app (this will create a new directory)
# Use echo "y" to automatically answer "yes" to the prompt if directory exists
if ! echo "y" | nova app create "$TEST_APP_NAME" -g python_app; then
    echo -e "${RED}❌ Failed to create test app with NOVA CLI${NC}"
    exit 1
fi

echo -e "${GREEN}✅ Test app created successfully${NC}"

# Go back to root directory
cd ..

# Check if the test app was created
if [ ! -d "$EXAMPLES_DIR/$TEST_APP_NAME" ]; then
    echo -e "${RED}❌ Test app directory not found${NC}"
    exit 1
fi

echo -e "${YELLOW}🔍 Analyzing generated app structure...${NC}"

# List the contents of the generated app
echo "Generated app structure:"
find "$EXAMPLES_DIR/$TEST_APP_NAME" -type f | head -20

# Check if we need to compare with existing app
if [ -d "$TEMP_DIR/existing_app_backup" ]; then
    echo -e "${YELLOW}🔍 Comparing new app with existing app...${NC}"

    # Compare the new app with the existing one, ignoring timestamp fields
    # Create filtered versions of both directories for comparison
    mkdir -p "$TEMP_DIR/filtered_existing" "$TEMP_DIR/filtered_new"

    # Copy directories and filter out timestamp fields
    cp -r "$TEMP_DIR/existing_app_backup"/* "$TEMP_DIR/filtered_existing/" 2>/dev/null || true
    cp -r "$EXAMPLES_DIR/$TEST_APP_NAME"/* "$TEMP_DIR/filtered_new/" 2>/dev/null || true

    # Remove or modify timestamp fields in .nova files
    find "$TEMP_DIR/filtered_existing" -name ".nova" -type f -exec sed -i.bak 's/generatedTs: [0-9]*/generatedTs: 0/g' {} \; 2>/dev/null || true
    find "$TEMP_DIR/filtered_new" -name ".nova" -type f -exec sed -i.bak 's/generatedTs: [0-9]*/generatedTs: 0/g' {} \; 2>/dev/null || true

    # Clean up backup files created by sed
    find "$TEMP_DIR/filtered_existing" -name "*.bak" -delete 2>/dev/null || true
    find "$TEMP_DIR/filtered_new" -name "*.bak" -delete 2>/dev/null || true

    # Now compare the filtered directories
    if diff -r "$TEMP_DIR/filtered_existing" "$TEMP_DIR/filtered_new" > "$TEMP_DIR/app_diff_output.txt" 2>&1; then
        echo -e "${GREEN}✅ No differences found between existing and new app (ignoring timestamps)${NC}"
        echo "The CLI generated identical content to the existing app."
    else
        echo -e "${RED}❌ Differences found between existing and new app!${NC}"
        echo "This suggests the CLI generated different content than expected."
        echo "Differences detected:"
        cat "$TEMP_DIR/app_diff_output.txt" | head -20
        echo ""
        echo -e "${YELLOW}💡 Use -f or --force flag to override the existing app${NC}"
        # Exit with a distinct status code to indicate that the app template
        # is out of sync but the CLI itself worked.
        exit 42
    fi
else
    # Compare with backup to see what changed (original behavior for new apps)
    echo -e "${YELLOW}🔍 Comparing with original examples directory...${NC}"

    # Use diff to compare directories
    if diff -r "$TEMP_DIR/examples_backup" "$EXAMPLES_DIR" > "$TEMP_DIR/diff_output.txt" 2>&1; then
        echo -e "${RED}❌ No differences found! The CLI didn't generate any new files.${NC}"
        echo "This suggests the CLI might not be working correctly."
        exit 1
    else
        echo -e "${GREEN}✅ Differences found - CLI generated new files as expected${NC}"
        echo "Changes detected:"
        cat "$TEMP_DIR/diff_output.txt" | head -20
    fi
fi

# Go back to root
cd ../..

echo -e "${GREEN}🎉 NOVA CLI SDK compatibility test completed successfully!${NC}"
echo -e "${GREEN}✅ The NOVA CLI correctly generates Python apps that use the NOVA SDK${NC}"

exit 0
