#!/bin/bash
# Carson Fujita

SRC_DIR="./src"
DEPLOY_DIR="/root/ros2_ws/src"

read -p "Do you want to deploy to "$DEPLOY_DIR"?" confirm
if [[ "$confirm" == "y" || "$confirm" == "Y" ]]; then
    cp -rf "$SRC_DIR"/* "$DEPLOY_DIR"
    echo "Copying contents of "$SRC_DIR" to "$DEPLOY_DIR"..."
    cd /root/ros2_ws
    colcon build --symlink-install --packages-select loon_e
    echo "Deployment complete."
else
    echo "Deployment cancelled."
fi