sudo systemctl status nvargus-daemon.service

source loon_hub # Source the loon_hub script for shared variables and functions


services_list=(
    nvargus-daemon
    IMU_Daemon
    zed_x_daemon
)

#######################################
# 
#######################################
has_camera_device_nodes() {
    return [[ -f "/dev/video0" ] && [ -f "/dev/video1" ]]
}


#######################################
# Determines if service is running
# Globals:
# Arguments:
#   $1 service
# Returns:
#   0 if yes
#   1 if no
#######################################
is_service_online() {
    local status=$(systemctl is-active $1 2>/dev/null)
    if [[ "$status" == "active" ]]; then
        return 0
    else
        return 1
    fi
}