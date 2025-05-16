#!/bin/sh
if [ "$(whoami)" != "root" ]; then
  echo "This script requires root privileges!"
  exit 1
fi

docker_compose_file_path=$(pwd)
systemd_service_file="iot2050-edu-robot.service"
tag="<docker_compose_file_path>"

echo "Deploying Docker Compose Services for Controlling Iot2050 Eduard"

echo "Add current file path \"$docker_compose_file_path\" to systemd service."
sed -i "s|$tag|$docker_compose_file_path|g" $systemd_service_file

echo "Installing systemd service on system"
cp $systemd_service_file /etc/systemd/system/
systemctl daemon-reload
systemctl enable $systemd_service_file
echo "Systemd service was installed and enabled, so it will start automatically at boot up."

echo "With following commands you can control the Iot2050 systemd service:"
echo "starting:           sudo systemctl start $systemd_service_file"
echo "stopping:           sudo systemctl stop $systemd_service_file"
echo "enable autostart:   sudo systemctl enable $systemd_service_file"
echo "disable autostart : sudo systemctl disable $systemd_service_file"
