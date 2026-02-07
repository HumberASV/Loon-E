FROM nvcr.io/nvidia/isaac/ros:aarch64-ros2_humble_4c0c55dddd2bbcc3e8d5f9753bee634c

ENV DEBIAN_FRONTEND=noninteractive
ENV CMAKE_PREFIX_PATH="/usr/local/zed:$CMAKE_PREFIX_PATH"
ENV ZED_DIR="/usr/local/zed"


# -----------------------------
# System dependencies
# -----------------------------
RUN apt-get update && apt-get install -y \
    wget \
    lsb-release \
    gnupg2 \
    udev \
    libusb-1.0-0 \
    libv4l-0 \
    libqt5xml5 \
    libqt5core5a \
    libqt5gui5 \
    libqt5widgets5 \
    libqt5opengl5 \
    zstd \
    git \
    kmod \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------
# Install ZED SDK (Jetson)
# IMPORTANT: --no-cuda flag
# -----------------------------
RUN wget -q \
    https://download.stereolabs.com/zedsdk/5.1/l4t36.4/jetsons \
    -O zed_sdk.run && \
    chmod +x zed_sdk.run && \
    ./zed_sdk.run -- silent skip_cuda && \
    rm zed_sdk.run

# -----------------------------
# Set up Python Zed API for the ZED SDK
# -----------------------------
WORKDIR /usr/local/zed
RUN python3 -m pip install --no-cache-dir --upgrade pip
RUN python3 get_python_api.py

# -----------------------------
# Install ROS 2 Dependencies
# Required for the ZED Wrapper to work
# -----------------------------
RUN apt-get update && apt-get install -y \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-diagnostic-updater \
    ros-humble-image-transport-plugins \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------
# Build the ZED ROS 2 Wrapper
# This is what actually creates the ROS nodes
# -----------------------------
WORKDIR /root/ros2_ws/src

# Clone the wrapper
RUN git clone https://github.com/stereolabs/zed-ros2-wrapper.git

# Install dependencies and build
WORKDIR /root/ros2_ws
RUN . /opt/ros/humble/setup.sh && \
    apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf /var/lib/apt/lists/*



RUN ln -sf /usr/lib/aarch64-linux-gnu/tegra/libv4l2.so.0 /usr/lib/aarch64-linux-gnu/libv4l2.so

# Add the workspace to your bashrc so it's ready when you log in
RUN echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc
