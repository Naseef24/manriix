#!/bin/bash
# check_copyright.sh
# This script checks if the copyright header exists in all Python files.

# Define the expected copyright header
HEADER="Copyright 2015 Open Source Robotics Foundation, Inc."

# Loop through all Python files in the repository
for file in $(find . -name "*.py"); do
  # Check if the header is present in the file
  if ! grep -q "$HEADER" "$file"; then
    echo "Missing copyright header in: $file"
    exit 1
  fi
done

echo "All Python files have the correct copyright header."
exit 0
