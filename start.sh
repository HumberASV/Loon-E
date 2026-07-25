#!/bin/bash

RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Running mtu.sh"
"$SCRIPT_DIR/mtu.sh"

echo "Checking daemon status"
failed=0

if sudo systemctl is-active --quiet nvargus-daemon.service; then
    echo -e "${GREEN}[OK]${NC} nvargus-daemon is active"
else
    echo -e "${RED}[FAIL]${NC} nvargus-daemon is not active"
    failed=1
fi

if sudo service zed_x_daemon status | grep -qi 'running'; then
    echo -e "${GREEN}[OK]${NC} zed_x_daemon is running"
else
    echo -e "${RED}[FAIL]${NC} zed_x_daemon is not running"
    failed=1
fi

if [ "$failed" -ne 0 ]; then
    echo -e "${RED}Daemon check failed. Try running ./reset.sh first.${NC}"
    exit 1
fi

echo "Sourcing install/setup.sh"
source "$SCRIPT_DIR/install/setup.sh"

echo "Launching SLAM"
ros2 launch loone slam_launch.py
