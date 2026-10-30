For each native ROS node running on the robot a systemd service is provided to autostart them.

# Idea
In systemd it is common to define wait services and also online target that could be used to boot up the robot in a specific order. We defined a `ros2.online.wait` service and `ros2.online.target` that are used by all EduArt systemd services on the robot to ensure correct startup order.

## Architecture

### ros2.online.wait Service
- **Purpose**: Checks if ROS2 is ready to use (daemon running, socket available)
- **Type**: oneshot
- **Behavior**: 
  - Executes `/usr/local/lib/ros2-online-wait.sh` script
  - Waits up to 30 seconds for ROS2 to become available
  - Sets `RemainAfterExit=yes` to indicate success
  - Retries with exponential backoff on failure
- **Logs**: Journal (systemd)

### ros2.online Target
- **Purpose**: Milestone/synchronization point for dependent services
- **Dependencies**: Requires successful completion of `ros2.online.wait.service`
- **Usage**: Other ROS2 services depend on this target to enforce startup order

## Installation

1. **Copy the scripts and services**:
```bash
sudo cp ros2-online-wait.sh /usr/local/lib/
sudo chmod +x /usr/local/lib/ros2-online-wait.sh

sudo cp ros2.online.wait.service /etc/systemd/system/
sudo cp ros2.online.target /etc/systemd/system/
```

2. **Reload systemd**:
```bash
sudo systemctl daemon-reload
```

3. **Enable services** (optional, auto-started by dependent services):
```bash
sudo systemctl enable ros2.online.wait.service
sudo systemctl enable ros2.online.target
```

## Usage in Other Services

To ensure ROS2 is online before starting your service, add the following to your service file:

```ini
[Unit]
After=ros2.online.target
Wants=ros2.online.target
```

Example:
```ini
[Unit]
Description=My ROS2 Node
Documentation=https://example.com

After=ros2.online.target
Wants=ros2.online.target

[Service]
Type=simple
ExecStart=/opt/ros/rolling/bin/ros2 run my_package my_node

[Install]
WantedBy=default.target
```

## Monitoring & Validation

### Quick Status Check
```bash
# Overall status
systemctl status ros2.online.wait.service
systemctl status ros2.online.target

# Short status
systemctl is-active ros2.online.wait.service  # Returns "active" or "inactive"
systemctl is-active ros2.online.target        # Returns "active" or "inactive"
```

### View Logs in Real-Time
```bash
# Live logs (follow mode)
journalctl -u ros2.online.wait.service -f

# Last 50 lines
journalctl -u ros2.online.wait.service -n 50

# Since system boot
journalctl -u ros2.online.wait.service -b

# Last 10 minutes
journalctl -u ros2.online.wait.service --since "10 min ago"

# Verbose output with timestamps
journalctl -u ros2.online.wait.service -o short-iso
```

### Manual Validation
```bash
# Check if script is executable
ls -la /usr/local/lib/ros2-online-wait.sh

# Run the script manually to test
/usr/local/lib/ros2-online-wait.sh

# Check ROS2 daemon status
ros2 daemon status

# Check eth0 IP address
ip addr show eth0

# Check multicast group membership
ip maddr show eth0

# Check ROS2 socket
ls -la /tmp/ros2_unix_domain_socket_*
```

### Systemd Details
```bash
# Show service file content
systemctl cat ros2.online.wait.service
systemctl cat ros2.online.target

# Show dependencies
systemctl show ros2.online.wait.service -p Wants -p After -p Requires

# List all units that want this target
systemctl list-dependencies ros2.online.target --reverse
```

## Troubleshooting

### Service won't start
```bash
# Check if service file is valid
systemd-analyze verify /etc/systemd/system/ros2.online.wait.service

# Check if script is executable and in correct path
file /usr/local/lib/ros2-online-wait.sh
which ros2
```

### Logs show "eth0 does not have an IP address"
```bash
# Check network interface status
ip link show eth0
ip addr show eth0

# Restart networking (if needed)
sudo systemctl restart networking
```

### Logs show "Waiting for ROS2 daemon"
```bash
# Check ROS2 daemon manually
ros2 daemon status

# Start daemon manually
ros2 daemon start

# Check if ROS environment is set
echo $ROS_DISTRO
echo $ROS_DOMAIN_ID
```

### Logs show "waiting for multicast group"
```bash
# Check multicast membership
ip maddr show eth0 | grep 239.255.0.1

# Manually trigger multicast by running a ROS2 node
ros2 node list

# Check sysctl settings
sysctl net.ipv4.igmp_max_memberships
```

### Service is stuck in infinite loop
- This is **expected behavior** - the service keeps retrying until conditions are met
- To manually stop it: `sudo systemctl stop ros2.online.wait.service`
- Check what condition is failing in the logs

