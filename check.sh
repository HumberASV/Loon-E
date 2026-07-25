#!/bin/bash

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

pass_count=0
fail_count=0

section() {
    echo -e "${CYAN}$1${NC}"
}

pass() {
    echo -e "${GREEN}[OK]${NC} $1"
    ((pass_count++))
}

fail() {
    echo -e "${RED}[FAIL]${NC} $1"
    ((fail_count++))
}

section "Checking /dev/video devices"
video_devices=$(ls /dev/video* 2>/dev/null)
if echo "$video_devices" | grep -q '/dev/video0' && echo "$video_devices" | grep -q '/dev/video1'; then
    pass "/dev/video0 and /dev/video1 present"
else
    fail "expected /dev/video0 and /dev/video1, found: ${video_devices:-none}"
fi

section "Checking nvargus-daemon status"
if sudo systemctl is-active --quiet nvargus-daemon.service; then
    pass "nvargus-daemon is active"
else
    fail "nvargus-daemon is not active"
    sudo systemctl status nvargus-daemon.service --no-pager
fi

section "Checking zed_x_daemon status"
if sudo service zed_x_daemon status | grep -qi 'running'; then
    pass "zed_x_daemon is running"
else
    fail "zed_x_daemon is not running"
    sudo service zed_x_daemon status
fi

section "Checking IMU_Daemon status"
# IMU_Daemon.service (Requires=zed_x_daemon.service) owns /tmp/imu_daemon.sock,
# the socket the ZED SDK connects to for motion sensor data. When zed_x_daemon
# gets stopped/restarted (e.g. by ZED_Diagnostic or reset.sh), systemd's
# Requires= cascades a stop to IMU_Daemon too, but does NOT bring it back up
# automatically afterwards - it's left dead until manually restarted, silently
# breaking IMU/motion sensor connectivity even though video/depth keep working.
if sudo systemctl is-active --quiet IMU_Daemon.service; then
    pass "IMU_Daemon is active"
else
    fail "IMU_Daemon is not active (motion sensors will fail to connect)"
    sudo systemctl status IMU_Daemon.service --no-pager
fi

section "Checking dmesg for zedx device"
if sudo dmesg | grep -qi zedx; then
    pass "zedx device found in dmesg"
else
    fail "no zedx entries found in dmesg"
fi

section "Checking for recent errors in nvargus-daemon log"
raw_errors=$(sudo journalctl -u nvargus-daemon.service --since "10 min ago" --no-pager | grep -i error)
# nvargus-daemon probes every camera slot declared in the DTB overlay at startup.
# Only ImagerGUID 0 and 1 are physically populated (the two ZED-X sensors); the
# remaining empty slots always log this same boilerplate error block, so filter
# it out here rather than treating it as a real failure.
recent_errors=$(echo "$raw_errors" | grep -vE \
    -e 'Model Name is NULL' \
    -e 'ModuleNotPresent' \
    -e 'Failed to init camera sub module v4l2_sensor' \
    -e 'PCL Open Failed' \
    -e 'Sensor could not be opened' \
    -e 'SCF: Error BadParameter' \
    -e 'serial no file already exists' \
    -e 'Failed ImagerGUID [2-9]')
if [ -n "$recent_errors" ]; then
    fail "recent errors found in nvargus-daemon log:"
    echo "$recent_errors"
else
    pass "no recent errors in nvargus-daemon log (ignoring known-benign empty camera slot probing)"
fi

section "Running ZED_Diagnostic --dmesg"
# ZED_Diagnostic prints its progress bar with carriage returns and wraps the
# final status in ANSI color codes; when this script's output isn't a tty
# those \r updates land in the captured text instead of being overwritten.
# Strip the color codes, turn each update into its own line, then keep only
# the last (final) status reported per diagnostic.
diag_output=$(sudo ZED_Diagnostic --dmesg 2>&1 | sed -E $'s/\x1b\\[[0-9;]*m//g' | tr '\r' '\n')
diag_lines=$(echo "$diag_output" | grep -E '^- ' | awk -F' : ' '
    { if (!($1 in seen)) order[++n] = $1; seen[$1] = $0 }
    END { for (i = 1; i <= n; i++) print seen[order[i]] }
')

if [ -z "$diag_lines" ]; then
    fail "ZED_Diagnostic produced no diagnostic results:"
    echo "$diag_output"
else
    while IFS= read -r line; do
        status=$(echo "$line" | sed -E 's/.*: *//' | awk '{print $1}')
        if [ "$status" = "OK" ]; then
            pass "${line#- }"
        else
            fail "${line#- }"
        fi
    done <<< "$diag_lines"
fi

echo ""
echo -e "${CYAN}Summary${NC}: ${GREEN}${pass_count} passed${NC}, ${RED}${fail_count} failed${NC}"

if [ "$fail_count" -ne 0 ]; then
    echo -e "${YELLOW}Some checks failed. Try running ./reset.sh or sudo ZED_Diagnostic --dmesg${NC}"
fi
