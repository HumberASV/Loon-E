#!/bin/bash

build_all(){
  source /opt/ros/humble/setup.bash &&  apt-get update && \
  rosdep update && \
  colcon build --parallel-workers 4 --symlink-install \
    --event-handlers console_direct+ \
    --base-paths src \
    --cmake-clean-cache \
    --cmake-args \
      -DCMAKE_BUILD_TYPE=Release \
      -DZED_SDK_ROOT=/usr/local/zed \
      -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs \
      -DCMAKE_CXX_FLAGS=\"-Wl,--allow-shlib-undefined\" \
      --no-warn-unused-cli
}

build_loone(){
  echo "Building only Loon-E packages..."
  colcon build --symlink-install --packages-select loon_e_coms loon_e_control loon_e_motor loon_e_planning loon_e_map && \
  source install/setup.bash
}

parse_arguements(){
  case "${1:-}" in
    -a|--all)
    build_all
    ;;
  -l|--loone)
    build_loone
    ;;
  *)
    echo "No valid option provided."
    exit 1
    ;;
  esac
}

parse_arguements "$@"