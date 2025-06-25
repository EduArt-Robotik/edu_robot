#!/bin/sh
if [ "$(whoami)" != "root" ]; then
  echo "This script requires root privileges!"
  exit 1
fi

systemd_service_file="iot2050-edu-robot.service"

# Stop and disable systemd service.
systemctl stop $systemd_service_file
systemctl disable $systemd_service_file

# Remove service from systemd system folder.
rm /etc/systemd/system/$systemd_service_file

# Restart daemon.
systemctl daemon-reload
