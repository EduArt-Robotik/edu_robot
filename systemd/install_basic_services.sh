#!/bin/bash
BIN_PATH=/usr/local/lib/eduart/
SERVICE_PATH=/etc/systemd/system/

mkdir -p $BIN_PATH
cp scripts/ros2_online_wait.sh $BIN_PATH
cp scripts/ros2_setup_environment.sh $BIN_PATH

chmod +x $BIN_PATH/ros2_online_wait.sh
chmod +x $BIN_PATH/ros2_setup_environment.sh

cp ros2.online.wait.service $SERVICE_PATH
cp ros2.setup.environment.service $SERVICE_PATH
cp ros2.online.target $SERVICE_PATH

systemctl daemon-reload

systemctl enable ros2.online.wait.service
systemctl enable ros2.setup.environment.service
systemctl enable ros2.online.target

echo "Basic ROS2 services installed and enabled."
