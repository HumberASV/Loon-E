#!/bin/bash

echo "Stopping nvargus"
sudo systemctl stop nvargus-daemon.service
echo "nvargus stopped"
sleep 2

echo "stopping zed"
sudo service zed_x_daemon stop
echo "zed stopped"
sleep 5

echo "clearing cache"
sudo rm -rf /tmp/argus_socket
echo "cache cleared"
