#!/usr/bin/env bash
# Installs (or removes) the official Stereolabs RViz2 object-detection display
# into the Loon-E workspace, so RViz can render the ZED wrapper's
# /zedx/zed_node/obj_det/objects boxes/labels/skeletons.
#
# Source: https://github.com/stereolabs/zed-ros2-examples
#   - rviz-plugin-zed-od  -> the actual rviz_plugin_zed_od Qt/rviz_common plugin
#   - zed_display_rviz2   -> ready-made .rviz configs + launch file that use it
# Docs: https://docs.stereolabs.com/docs/integrations/ros-2/data-display-with-r-viz-2
#
# Usage:
#   ./install-rviz.sh [install|remove]   (default: install)
set -euo pipefail

REPO_URL="https://github.com/stereolabs/zed-ros2-examples.git"
REPO_REF="v5.4.0"
PACKAGES=(rviz-plugin-zed-od zed_display_rviz2)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$SCRIPT_DIR/Loon-E"
SRC_DIR="$WS_ROOT/src"

ACTION="${1:-install}"

remove_packages() {
    for pkg in "${PACKAGES[@]}"; do
        if [[ -d "$SRC_DIR/$pkg" ]]; then
            echo "Removing $SRC_DIR/$pkg"
            rm -rf "$SRC_DIR/$pkg"
        fi
    done
    # Clean stale build artifacts so colcon doesn't trip over deleted packages.
    for pkg_cmake in rviz_plugin_zed_od zed_display_rviz2; do
        rm -rf "$WS_ROOT/build/$pkg_cmake" "$WS_ROOT/install/$pkg_cmake"
    done
    echo "Done. Re-run 'colcon build' in $WS_ROOT to refresh the install space."
}

install_packages() {
    if [[ ! -d "$SRC_DIR" ]]; then
        echo "Expected workspace src dir not found: $SRC_DIR" >&2
        exit 1
    fi

    TMP_DIR="$(mktemp -d)"
    trap 'rm -rf "$TMP_DIR"' EXIT

    echo "Fetching ${PACKAGES[*]} from $REPO_URL @ $REPO_REF"
    git clone --quiet --depth 1 --branch "$REPO_REF" --filter=blob:none \
        --sparse "$REPO_URL" "$TMP_DIR/zed-ros2-examples"
    git -C "$TMP_DIR/zed-ros2-examples" sparse-checkout set "${PACKAGES[@]}" >/dev/null

    for pkg in "${PACKAGES[@]}"; do
        rm -rf "$SRC_DIR/$pkg"
        cp -r "$TMP_DIR/zed-ros2-examples/$pkg" "$SRC_DIR/$pkg"
        echo "Installed $SRC_DIR/$pkg"
    done

    cat <<EOF

Next steps:
  cd $WS_ROOT
  colcon build --packages-select rviz_plugin_zed_od zed_display_rviz2
  source install/setup.bash

  # Start your real stack first (this is what wires up the custom ONNX model):
  ros2 launch loone bringup.launch.py   # or slam_launch.py

  # Then, in another shell, open RViz against it. start_zed_node:=False is
  # required -- otherwise this launches a second, unconfigured zed_camera.launch.py
  # with no custom_object_detection_config_path/param_overrides, and object
  # detection will fail with "'object_detection.custom_onnx_file' is empty".
  ros2 launch zed_display_rviz2 display_zed_cam.launch.py \\
      start_zed_node:=False camera_name:=zedx camera_model:=zedx
EOF
}

case "$ACTION" in
    install) install_packages ;;
    remove|uninstall) remove_packages ;;
    *)
        echo "Usage: $0 [install|remove]" >&2
        exit 1
        ;;
esac
