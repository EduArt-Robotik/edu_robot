#!/bin/sh
if [ "$(whoami)" != "root" ]; then
  echo "This script requires root privileges!"
  exit 1
fi

docker_compose_file_path=$(pwd)
systemd_service_file="ipcbx21a-edu-robot.service"
tag="<docker_compose_file_path>"

echo "Deploying Docker Compose Services for Controlling IPCBX21a Eduard"

# First installing systemd service on system.
echo "Installing systemd service on system"
cp $systemd_service_file /etc/systemd/system/

# Modify systemd service so it runs the correct docker compose file.
echo "Add current file path \"$docker_compose_file_path\" to systemd service."
sed -i "s|$tag|$docker_compose_file_path|g" /etc/systemd/system/$systemd_service_file

# Do use docker compose command to install Docker image.
echo "Pulling docker image..."
docker compose up -d
docker compose down

# Starting up service.
systemctl daemon-reload
systemctl start $systemd_service_file
systemctl enable $systemd_service_file
echo "Systemd service was installed and enabled, so it will start automatically at boot up."

# Printing info text.
echo "With following commands you can control the IPCBX21a systemd service:"
echo "starting:           sudo systemctl start $systemd_service_file"
echo "stopping:           sudo systemctl stop $systemd_service_file"
echo "enable autostart:   sudo systemctl enable $systemd_service_file"
echo "disable autostart : sudo systemctl disable $systemd_service_file"
