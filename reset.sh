#!/bin/bash
echo "Stopping nvargus"
sudo systemctl stop nvargus-daemon.service
echo "nvargus stopped"
sleep 2
echo "stopping IMU_Daemon"
sudo systemctl stop IMU_Daemon.service
echo "IMU_Daemon stopped"
sleep 1
echo "stopping zed"
sudo service zed_x_daemon stop
echo "zed stopped"
sleep 5
echo "clearing cache"
sudo rm -rf /tmp/argus_socket
echo "cache cleared"
sleep 1
echo "starting service nvargus"
sudo systemctl start nvargus-daemon.service
sleep 5

if sudo systemctl is-active --quiet nvargus-daemon.service; then
    echo "nvargus-daemon is active"
else
    echo "ERROR: nvargus-daemon failed to start"
    sudo systemctl status nvargus-daemon.service --no-pager
fi

echo "starting zed"
sudo service zed_x_daemon start
sleep 10

if sudo service zed_x_daemon status | grep -qi 'running'; then
    echo "zed_x_daemon is running"
else
    echo "ERROR: zed_x_daemon failed to start"
    sudo service zed_x_daemon status
fi

echo "starting IMU_Daemon"
sudo systemctl start IMU_Daemon.service
sleep 2

if sudo systemctl is-active --quiet IMU_Daemon.service; then
    echo "IMU_Daemon is active"
else
    echo "ERROR: IMU_Daemon failed to start"
    sudo systemctl status IMU_Daemon.service --no-pager
fi

input=$(ZED_Explorer -a 2>&1)

if echo "$input" | grep -q 'State :  "AVAILABLE"'; then
    echo "Camera is available!"
else
    echo "Camera is not available."
fi
