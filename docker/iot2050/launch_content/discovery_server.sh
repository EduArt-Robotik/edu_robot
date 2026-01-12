#!/usr/bin/env bash
# start the discovery server for ROS 2 communication if wanted

# checks and extracts IP address and optional port from input string
extract_ip_and_port() {
  local input="$1"
  local ip port

  # extract port and ip address
  if [[ "$input" == *:* ]]; then
    ip="${input%%:*}"
    port="${input#*:}"
    # Port darf nicht leer sein und muss numerisch 1-65535 sein
    [[ -n "$port" ]] || return 1
    [[ "$port" =~ ^[0-9]+$ ]] || return 1
    (( port >= 1 && port <= 65535 )) || return 1
  else
    # no valid string given
    return 1
  fi

  # validate ip address string
  if [[ $ip =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]]; then
    IFS='.' read -r a b c d <<< "$ip"
    for oct in $a $b $c $d; do
      [[ "$oct" =~ ^[0-9]+$ ]] || return 1
      (( oct >= 0 && oct <= 255 )) || return 1
    done
  else
    return 1
  fi

  # set extracted values
  EXTRACTED_IP="$ip"
  EXTRACTED_PORT="$port"
  return 0
}

if [ -z "$ROS_DISCOVERY_SERVER" ]; then
  echo "ROS_DISCOVERY_SERVER is not set. Uses default discovery mechanism."
  exit 0
fi

if ! extract_ip_and_port "$ROS_DISCOVERY_SERVER"; then
  echo "Invalid ROS_DISCOVERY_SERVER format. Use IP_ADDRESS:PORT"
  exit 1
fi

# starting up the discovery server with given IP address
echo "Starting ROS 2 Discovery Server..."
echo "on ip address = $EXTRACTED_IP"
echo "using port = $EXTRACTED_PORT"

fastdds discovery --server-id 0 --ip-address=$EXTRACTED_IP --port=$EXTRACTED_PORT
