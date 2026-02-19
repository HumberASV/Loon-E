FROM nvcr.io/nvidia/isaac/ros:aarch64-ros2_humble_4c0c55dddd2bbcc3e8d5f9753bee634c

ENV DEBIAN_FRONTEND=noninteractive
ENV CMAKE_PREFIX_PATH="/usr/local/zed:$CMAKE_PREFIX_PATH"
ENV ZED_DIR="/usr/local/zed"
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# -----------------------------
# 1. System dependencies & Isaac ROS Repo
# -----------------------------
RUN rm -f /etc/apt/sources.list.d/yarn.list && \
    apt-get update && apt-get install -y \
    wget curl gnupg2 lsb-release ca-certificates jq git zstd udev \
    libusb-1.0-0 libv4l-0 kmod \
    libqt5xml5 libqt5core5a libqt5gui5 libqt5widgets5 libqt5opengl5 \
    && curl -fsSL https://isaac.download.nvidia.com/isaac-ros/repos.key | gpg --dearmor -o /usr/share/keyrings/nvidia-isaac-ros.gpg \
    && echo "deb [signed-by=/usr/share/keyrings/nvidia-isaac-ros.gpg] https://isaac.download.nvidia.com/isaac-ros/release-3 jammy release-3" | tee /etc/apt/sources.list.d/nvidia-isaac-ros.list \
    && apt-get update && apt-get install -y \
    ros-humble-isaac-ros-yolov8 \
    ros-humble-isaac-ros-dnn-image-encoder \
    ros-humble-isaac-ros-tensor-rt \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------
# 2. ZED SDK Installation
# -----------------------------
RUN wget -q https://download.stereolabs.com/zedsdk/5.2/l4t36.4/jetsons -O zed_sdk.run && \
    chmod +x zed_sdk.run && ./zed_sdk.run -- silent skip_cuda && rm zed_sdk.run

RUN python3 -m pip install --no-cache-dir numpy
RUN sudo ln -s /usr/local/lib/python3.10/dist-packages/numpy/_core/include /usr/local/lib/python3.10/dist-packages/numpy/core/include

# -----------------------------
# 3. Clone & Build ZED ROS 2 Wrapper
# -----------------------------
robo@ubuntu:~/Desktop/project_files$ scp Dockerfile sven@artemis:/tmp/Dockerfile
ssh: connect to host artemis port 22: Connection refused
lost connection
robo@ubuntu:~/Desktop/project_files$ cat Dockerfile 
FROM nvcr.io/nvidia/isaac/ros:aarch64-ros2_humble_4c0c55dddd2bbcc3e8d5f9753bee634c

ENV DEBIAN_FRONTEND=noninteractive
ENV CMAKE_PREFIX_PATH="/usr/local/zed:$CMAKE_PREFIX_PATH"
ENV ZED_DIR="/usr/local/zed"
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# -----------------------------
# 1. System dependencies & Isaac ROS Repo
# -----------------------------
RUN rm -f /etc/apt/sources.list.d/yarn.list && \
    apt-get update && apt-get install -y \
    wget curl gnupg2 lsb-release ca-certificates jq git zstd udev \
    libusb-1.0-0 libv4l-0 kmod \
    libqt5xml5 libqt5core5a libqt5gui5 libqt5widgets5 libqt5opengl5 \
    && curl -fsSL https://isaac.download.nvidia.com/isaac-ros/repos.key | gpg --dearmor -o /usr/share/keyrings/nvidia-isaac-ros.gpg \
    && echo "deb [signed-by=/usr/share/keyrings/nvidia-isaac-ros.gpg] https://isaac.download.nvidia.com/isaac-ros/release-3 jammy release-3" | tee /etc/apt/sources.list.d/nvidia-isaac-ros.list \
    && apt-get update && apt-get install -y \
    ros-humble-isaac-ros-yolov8 \
    ros-humble-isaac-ros-dnn-image-encoder \
    ros-humble-isaac-ros-tensor-rt \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------
# 2. ZED SDK Installation
# -----------------------------
RUN wget -q https://download.stereolabs.com/zedsdk/5.2/l4t36.4/jetsons -O zed_sdk.run && \
    chmod +x zed_sdk.run && ./zed_sdk.run -- silent skip_cuda && rm zed_sdk.run

RUN python3 -m pip install --no-cache-dir numpy
RUN sudo ln -s /usr/local/lib/python3.10/dist-packages/numpy/_core/include /usr/local/lib/python3.10/dist-packages/numpy/core/include

# -----------------------------
# 3. Clone & Build ZED ROS 2 Wrapper
# -----------------------------
RUN mkdir -p /root/ros2_ws/src
WORKDIR /root/ros2_ws/src
RUN git clone --recursive https://github.com/stereolabs/zed-ros2-wrapper.git

WORKDIR /root/ros2_ws

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
  apt-get update && \
  rosdep update && \
  rosdep install --from-paths src --ignore-src -r -y && \
  colcon build --parallel-workers 4 --symlink-install \
    --event-handlers console_direct+ \
    --base-paths src \
    --packages-up-to zed_wrapper \
    --cmake-clean-cache \
    --cmake-args \
      -DCMAKE_BUILD_TYPE=Release \
      -DZED_SDK_ROOT=/usr/local/zed \
      -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs \
      -DCMAKE_CXX_FLAGS=\"-Wl,--allow-shlib-undefined\" \
      --no-warn-unused-cli && \
  rm -rf /var/lib/apt/lists/*"


# -----------------------------
# 5. NGC Assets & Final Setup
# -----------------------------
ENV MAJOR_VERSION=3
ENV MINOR_VERSION=1
RUN VERSION_REQ_URL="https://catalog.ngc.nvidia.com/api/resources/versions?orgName=nvidia&teamName=isaac&name=isaac_ros_yolov8_assets&isPublic=true&pageNumber=0&pageSize=100&sortOrder=CREATED_DATE_DESC" && \
    LATEST_VERSION_ID=$(curl -s -H "Accept: application/json" "$VERSION_REQ_URL" | jq -r ".recipeVersions[] | .versionId as \$v | \$v | select(test(\"^\\\\d+\\\\.\\\\d+\\\\.\\\\d+$\")) | split(\".\") | {major: .[0]|tonumber, minor: .[1]|tonumber, patch: .[2]|tonumber} | select(.major == $MAJOR_VERSION and .minor <= $MINOR_VERSION) | \$v" | sort -V | tail -n 1) && \
    if [ -n "$LATEST_VERSION_ID" ]; then \
        mkdir -p /root/ros2_ws/isaac_ros_assets && \
        curl -LO --request GET "https://api.ngc.nvidia.com/v2/resources/nvidia/isaac/isaac_ros_yolov8_assets/versions/$LATEST_VERSION_ID/files/quickstart.tar.gz" && \
        tar -xf quickstart.tar.gz -C /root/ros2_ws/isaac_ros_assets && \
        rm quickstart.tar.gz; \
    fi

RUN ln -sf /usr/lib/aarch64-linux-gnu/tegra/libv4l2.so.0 /usr/lib/aarch64-linux-gnu/libv4l2.so
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc

WORKDIR /root/ros2_ws
