#!/usr/bin/env bash
# start the zenoh router for ROS 2 communication if wanted
# the router allows communication between different networks

if [[ ${RMW_IMPLEMENTATION} != "rmw_zenoh_cpp" ]]; then
  # no need to launch a zenoh router
  exit 0
fi

# launching zenoh router
echo "Detected \"rmw_zenoh_cpp\" as RMW implementation was selected."
echo "Starting zenoh router for ROS 2 communication..."

export ZENOH_ROUTER_CONFIG_URI="./zenoh_router_config.json5"
ros2 run rmw_zenoh_cpp rmw_zenohd
