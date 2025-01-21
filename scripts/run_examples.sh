#!/usr/bin/env bash

# Exit immediately if a command exits with a non-zero status.
set -e

# -- Define blacklist of scripts to skip --
BLACKLIST=("07_auth0_with_device_code.py")

# -- Loop through all Python scripts in /examples and run them unless blacklisted --
for script in examples/*.py; do
  # Extract just the filename (e.g., '01_basic.py')
  filename=$(basename "$script")

  # Check if current filename is in the blacklist
  if [[ " ${BLACKLIST[@]} " =~ " ${filename} " ]]; then
    echo "Skipping blacklisted script: $filename"
    continue
  fi

  echo "Running $filename ..."
  PYTHONPATH=. poetry run python "$script"
  echo
done
