#!/bin/bash
# Script to check if ROS2 is ready/online
# This script is executed by ros2.online.wait service
# Exit with 0 if ROS2 is online, non-zero otherwise

# Don't exit on errors - we want to retry indefinitely
set +e

# Load ROS2 environment
source /home/user/ros2_jazzy/install/setup.bash

# Configuration
NETWORK_INTERFACE="eth0"
MULTICAST_GROUP="239.255.0.1"
SLEEP_INTERVAL=1
ATTEMPT=0

# Function to check if eth0 has an IP address
check_eth0_ip() {
    # Check if interface exists and is UP
    if ! ip link show "$NETWORK_INTERFACE" 2>/dev/null | grep -q "UP"; then
        return 1
    fi
    
    # Check if interface has a valid IPv4 address (not 0.0.0.0 and not loopback)
    local ip=$(ip addr show "$NETWORK_INTERFACE" 2>/dev/null | grep "inet " | awk '{print $2}' | cut -d/ -f1)
    
    # Check if IP is set and not 0.0.0.0
    if [[ -z "$ip" ]] || [[ "$ip" == "0.0.0.0" ]] || [[ "$ip" == "127."* ]]; then
        return 1
    fi
    
    echo "eth0 has valid IP address: $ip"
    return 0
}

# Function to start ROS2 daemon
start_ros2_daemon() {
    echo "Starting ROS2 daemon..."
    ros2 daemon start
    sleep 1
    return 0
}

# Function to check if ROS2 daemon is running
check_ros2_daemon() {
    # Exit code is always 0, so we check the output instead
    ros2 daemon status
    local status_output=$(ros2 daemon status 2>&1)
    if echo "$status_output" | grep -q "The daemon is not running"; then
        return 1
    fi
    # If output doesn't contain "not running", the daemon is running
    return 0
}

# Function to check if eth0 is in multicast group
check_multicast_membership() {
    # Check if eth0 is joined to the multicast group using ip maddr
    if ip maddr show "$NETWORK_INTERFACE" 2>/dev/null | grep -q "$MULTICAST_GROUP"; then
        echo "eth0 is in multicast group $MULTICAST_GROUP"
        return 0
    fi
    return 1
}

echo "=== ROS2 Online Check ==="
echo "Interface: $NETWORK_INTERFACE"
echo "Multicast Group: $MULTICAST_GROUP"
echo "Waiting indefinitely until all conditions are met..."
echo

# Infinite loop - keep trying until all conditions are met
while true; do
    ((ATTEMPT++))
    
    # Step 1: Check eth0 has IP
    if ! check_eth0_ip; then
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] Attempt $ATTEMPT: Waiting for eth0 IP address..."
        sleep $SLEEP_INTERVAL
        continue
    fi
    
    # Step 2: Start ROS2 daemon
    if ! start_ros2_daemon; then
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] Attempt $ATTEMPT: Failed to start ROS2 daemon, retrying..."
        sleep $SLEEP_INTERVAL
        continue
    fi
    
    # Step 3: Check ROS2 daemon and socket
    if ! check_ros2_daemon; then
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] Attempt $ATTEMPT: Waiting for ROS2 daemon and socket..."
        sleep $SLEEP_INTERVAL
        continue
    fi
    
    # Step 4: Check multicast membership
    if ! check_multicast_membership; then
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] Attempt $ATTEMPT: Daemon running, waiting for multicast group..."
        sleep $SLEEP_INTERVAL
        continue
    fi
    
    # All conditions met!
    echo
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] SUCCESS: ROS2 is online and multicast configured"
    echo "Completed after $ATTEMPT attempts"
    exit 0
done
