# `colcon` Setup

## Background

`colcon` is an iteration on the ROS build tools `catkin_make`, `catkin_make_isolated`, `catkin_tools` and `ament_tools`. For more information on the design of `colcon` see this [document](https://design.ros2.org/articles/build_tool.html).

The source code can be found in the [colcon GitHub organization](https://github.com/colcon).

### Requirements

Ensure you already installed ROS 2.
If you have not installed ROS 2 follow the [installation instructions](https://docs.ros.org/en/humble/Installation.html)

## Installation Linux

You can install `colcon` through the `apt` package manager with the command:

```bash
sudo apt install python3-colcon-common-extensions
```

## Installation macOs

You can install colcon with your Python `pip` module with the command:

```zsh
python3 -m pip install colcon-common-extensions
```

## Installation Windows

You can install colcon with your Python `pip` module with the command:

```powershell
pip install -U colcon-common-extensions
```
