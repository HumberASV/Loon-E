# Node Configuration

All node parameters are defined in `src/loone/config/config.yaml` and loaded at runtime via ROS 2's parameter system. Each section corresponds to a node namespace. Parameters are declared with defaults in code, so the file only needs to contain values you want to override.

To launch with the config file:

```bash
ros2 launch loone <launch_file>.py --ros-args --params-file src/loone/config/config.yaml
```

---

## `/led`

Controls the GPIO pins for the RGB LED on the Jetson board.

| Parameter   | Default | Description                        |
|-------------|---------|------------------------------------|
| `red_pin`   | `15`    | GPIO board pin for the red channel |
| `green_pin` | `32`    | GPIO board pin for the green channel |
| `blue_pin`  | `33`    | GPIO board pin for the blue channel |
| `data_pin`  | `40`    | GPIO board pin for data input      |

---

## `/mapping`

Controls the occupancy grid map dimensions and starting position.

| Parameter        | Default | Description                                           |
|------------------|---------|-------------------------------------------------------|
| `local_w`        | `20`    | Local map width in meters                             |
| `local_l`        | `20`    | Local map length in meters                            |
| `global_w`       | `50`    | Global map width in meters                            |
| `global_l`       | `50`    | Global map length in meters                           |
| `res`            | `0.5`   | Map resolution in meters per cell                     |
| `start_position` | `4`     | Starting quadrant/position (0.5, 1, 1.5 … 4)         |

`start_position` values correspond to compass positions around the global map:

| Value | Position        |
|-------|-----------------|
| `0.5` | Positive Y axis |
| `1`   | Quadrant 1      |
| `1.5` | Positive X axis |
| `2`   | Quadrant 2      |
| `2.5` | Negative Y axis |
| `3`   | Quadrant 3      |
| `3.5` | Negative X axis |
| `4`   | Quadrant 4      |

---

## `/motor`

Controls the PCA9685 PWM driver and PID heading controller.

| Parameter   | Default | Description                                          |
|-------------|---------|------------------------------------------------------|
| `freq`      | `50`    | PCA9685 PWM frequency in Hz                          |
| `factor`    | `0.75`  | Initial propeller throttle factor (0.0–1.0)          |
| `center`    | `0.55`  | Rudder center fraction (neutral/0°)                  |
| `kp`        | `1.0`   | PID proportional gain                                |
| `ki`        | `0.0`   | PID integral gain                                    |
| `kd`        | `0.0`   | PID derivative gain                                  |
| `max`       | `45.0`  | Max heading error / PID output clamp in degrees      |
| `prop_min`  | `1120`  | Propeller servo minimum pulse width in µs            |
| `prop_max`  | `1880`  | Propeller servo maximum pulse width in µs            |
| `rudder_min`| `1220`  | Rudder servo minimum pulse width in µs               |
| `rudder_max`| `1820`  | Rudder servo maximum pulse width in µs               |

---

## `/path`

| Parameter   | Default      | Description                              |
|-------------|--------------|------------------------------------------|
| `path_file` | `"path.txt"` | Path to the file containing waypoint coordinates |

---

## `/phone`

Controls the UDP tunnel between the phone and the onboard computer.

| Parameter       | Default       | Description                                    |
|-----------------|---------------|------------------------------------------------|
| `phone_port`    | `5000`        | Port on the phone to forward from              |
| `computer_port` | `5000`        | Port on the computer to receive on             |
| `host`          | `"127.0.0.1"` | Host address for the phone node (default: localhost) |

---

## `/task`

| Parameter      | Default | Description                                  |
|----------------|---------|----------------------------------------------|
| `timer_period` | `0.25`  | Timer period for the task node in seconds    |
