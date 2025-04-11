#!/bin/sh

uv run download-models

# Copy directories from parent folder
PARENT_DIR="../.."
DIRS_TO_COPY=("models" "nova_rerun_bridge")
FILES_TO_COPY=("pyproject.toml" "uv.lock" "README.md")

for dir in "${DIRS_TO_COPY[@]}"; do
    if [ -d "$PARENT_DIR/$dir" ]; then
        echo "Copying $dir..."
        rm -rf "$dir"
        cp -r "$PARENT_DIR/$dir" .
    else
        echo "Warning: Directory $dir not found in parent directory"
    fi
done

for file in "${FILES_TO_COPY[@]}"; do
    if [ -f "$PARENT_DIR/$file" ]; then
        echo "Copying $file..."
        cp "$PARENT_DIR/$file" .
    else
        echo "Warning: File $file not found in parent directory"
    fi
done

# Run Nova app installation
echo "Running Nova app installation..."
nova app install .

# Clean up copied directories and files
for dir in "${DIRS_TO_COPY[@]}"; do
    echo "Removing $dir..."
    rm -rf "$dir"
done

for file in "${FILES_TO_COPY[@]}"; do
    echo "Removing $file..."
    rm -f "$file"
done

echo "Build process completed"
