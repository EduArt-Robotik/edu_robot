#!/bin/bash
# Script to generate ROS2 environment file
# This is executed by ros2-setup-environment.service at boot
# It extracts the ROS2 environment variables and saves them to /etc/ros2.env

set -e

# Try to source ROS2 setup files in order of preference
ROS2_SETUP_FOUND=0

for setup_path in \
    "/opt/ros/rolling/setup.bash" \
    "/opt/ros/jazzy/setup.bash" \
    "/opt/ros/kilted/setup.bash" \
    "/home/user/ros2_jazzy/install/setup.bash"; do
    
    if [[ -f "$setup_path" ]]; then
        echo "Found ROS2 setup at: $setup_path"
        source "$setup_path" 2>/dev/null
        ROS2_SETUP_FOUND=1
        break
    fi
done

if [[ $ROS2_SETUP_FOUND -eq 0 ]]; then
    echo "ERROR: Could not find any ROS2 setup.bash file"
    exit 1
fi

# Generate the environment file
mkdir -p /etc/eduart
ENV_FILE="/etc/eduart/ros2.env"
echo "Generating ROS2 environment file at: $ENV_FILE"

# Extract ROS2-related environment variables
env | grep -E "^(PATH|AMENT|PYTHONPATH|ROS|LD_LIBRARY_PATH|COLCON)" | sort > "$ENV_FILE.tmp"

# Move the temp file to the final location (atomic operation)
mv "$ENV_FILE.tmp" "$ENV_FILE"
chmod 644 "$ENV_FILE"

echo "Successfully created $ENV_FILE"
echo "Content:"
cat "$ENV_FILE"

exit 0
