"""PCA9685 driver: JointState commands -> servo PWM over I2C.

This node is the bottom of the chained-controls stack. It is the ONLY node that
touches the I2C bus / PCA9685 hardware:

    ros2_control (topic_based_ros2_control / TopicBasedSystem)
        --> /asv/joint_commands (sensor_msgs/JointState)  [command interface values]
        --> [THIS NODE] fraction -> servo channel over I2C
        --> PCA9685  (ch0 prop_l, ch1 prop_r, ch2 rudder)
    and it echoes measured state back:
        --> /asv/joint_states (sensor_msgs/JointState)  [state interface values]

It deliberately reuses the proven PCA9685 setup from motor.py (same channels,
same pulse-width limits, same adafruit libraries). The control math that used to
live in motor.py now lives upstream (nav2 + thrust_mixer), so this node is a thin,
safe hardware shim.

Incoming values are normalized servo fractions in [0, 1] (see thrust_mixer.py):
    * propellers: 0.0 reverse, 0.5 neutral, 1.0 forward
    * rudder:     0.0 / center (~0.55) / 1.0
Because the ros2_control command interface is declared as "position" in the URDF,
the fractions arrive in JointState.position. (If you switch the URDF interface to
"velocity", read msg.velocity instead -- see JOINT_COMMAND_FIELD below.)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
import busio
import board
from adafruit_ina3221 import INA3221
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685


class Pca9685Driver(Node):
    """Drive two propeller ESCs and one rudder servo on a PCA9685 from JointState commands."""

    # Valid frequency range for PCA9685 (Hz) -- copied from motor.py.
    PCA_FREQ_MIN = 24
    PCA_FREQ_MAX = 1526

    # Absolute pulse width limits enforced by hardware (microseconds) -- copied from motor.py.
    PULSE_MIN_LIMIT = 500
    PULSE_MAX_LIMIT = 2500

    # Which JointState field carries the command. Must match the URDF command interface type
    # ("position" -> .position, "velocity" -> .velocity). See loone_asv.urdf.xacro.
    JOINT_COMMAND_FIELD = 'position'

    def __init__(self) -> None:
        super().__init__('pca9685_driver')

        # ---- Parameters (defaults mirror config.yaml /motor block and motor.py) ----
        self.declare_parameter('timer_period', 0.25) # Timer period to publish battery data
        self.declare_parameter('freq', 50)          # PCA9685 PWM frequency (Hz)
        self.declare_parameter('prop_min', 1120)    # propeller servo min pulse (us)
        self.declare_parameter('prop_max', 1880)    # propeller servo max pulse (us)
        self.declare_parameter('rudder_min', 1220)  # rudder servo min pulse (us)
        self.declare_parameter('rudder_max', 1820)  # rudder servo max pulse (us)
        self.declare_parameter('prop_neutral', 0.5)   # propeller fraction = no thrust
        self.declare_parameter('rudder_center', 0.55)  # rudder fraction = straight
        # Dead-man: if no command arrives within this many seconds, go neutral.
        self.declare_parameter('cmd_timeout', 0.5)

        timer_period = self.get_parameter('timer_period').value
        freq = self.get_parameter('freq').value
        prop_min = self.get_parameter('prop_min').value
        prop_max = self.get_parameter('prop_max').value
        rudder_min = self.get_parameter('rudder_min').value
        rudder_max = self.get_parameter('rudder_max').value
        self.prop_neutral = self.get_parameter('prop_neutral').value
        self.rudder_center = self.get_parameter('rudder_center').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value

        # ---- Hardware bring-up (same sequence as motor.py) ----
        self._init_busio(freq)
        self._init_servos(prop_min, prop_max, rudder_min, rudder_max)

        # Map ros2_control joint names -> the servo object on each PCA9685 channel.
        # Channel assignment is unchanged from motor.py: ch0 prop_l, ch1 prop_r, ch2 rudder.
        # These names MUST match the joints declared in loone_asv.urdf.xacro / ros2_control.yaml.
        self.joint_to_servo = {
            'prop_l_joint': self.prop_l,
            'prop_r_joint': self.prop_r,
            'rudder_joint': self.rudder,
        }
        # Per-joint neutral used at startup, on stale commands, and on shutdown.
        self.joint_neutral = {
            'prop_l_joint': self.prop_neutral,
            'prop_r_joint': self.prop_neutral,
            'rudder_joint': self.rudder_center,
        }

        # ---- ROS wiring ----
        # Command topic + state topic names MUST match the TopicBasedSystem params in the URDF.
        # Set up before the initial neutral apply below, since _apply publishes state.
        # QoS must match TopicBasedSystem's publisher (rclcpp::SensorDataQoS, best-effort) --
        # a reliable subscriber is incompatible with a best-effort publisher and silently never
        # receives anything (this is why the boat never moved: joint_commands was DOA).
        self.cmd_sub = self.create_subscription(
            JointState, '/asv/joint_commands', self.command_callback, qos_profile_sensor_data)
        self.state_pub = self.create_publisher(JointState, '/asv/joint_states', 10)
        self.battery_pub = self.create_publisher(Float32MultiArray, 'battery', 10)

        # Start every channel at its neutral so the boat does not lurch on boot.
        self._apply(dict(self.joint_neutral))

        self.last_cmd_time = self.get_clock().now()
        # Watchdog: periodically check for stale commands and re-publish state.
        self.timer = self.create_timer(0.1, self.watchdog)
        self.timer = self.create_timer(timer_period, self.publish_battery)

        self.get_logger().info('pca9685_driver ready, channels at neutral.')

    # ------------------------------------------------------------------ hardware setup
    def _init_busio(self, freq) -> None:
        """Initialize the PCA9685 PWM driver over I2C (ported from motor.py)."""
        if not (self.PCA_FREQ_MIN <= freq <= self.PCA_FREQ_MAX):
            self.get_logger().error(
                f"PCA9685 frequency {freq} Hz out of range "
                f"[{self.PCA_FREQ_MIN}, {self.PCA_FREQ_MAX}] Hz.")
            raise ValueError(f"Invalid PCA9685 frequency: {freq}")

        try:
            i2c = busio.I2C(board.SCL, board.SDA)
        except Exception as e:
            self.get_logger().error(f"Failed to initialize I2C bus (check SDA/SCL): {e}")
            raise

        try:
            self.pca = PCA9685(i2c, address = 64)
        except Exception as e:
            self.get_logger().error(f"PCA9685 not found on I2C bus (check wiring/address 0x40): {e}")
            raise

        try:
            self.ina = INA3221(i2c, address = 65, enable = [0, 1, 2])
        except Exception as e:
            self.get_logger().error(f"INA3221 not found on I2C bus (check wiring/address 0x41): {e}")
            raise
        
        self.pca.frequency = freq
        self.get_logger().info(f"PCA9685 initialized at {freq} Hz.")

    def _validate_pulse_range(self, min_pulse, max_pulse, channel_name) -> None:
        """Validate PWM pulse widths are ordered and within hardware limits (ported from motor.py)."""
        if min_pulse >= max_pulse:
            raise ValueError(
                f"{channel_name}: min_pulse ({min_pulse}) must be < max_pulse ({max_pulse}).")
        if min_pulse < self.PULSE_MIN_LIMIT or max_pulse > self.PULSE_MAX_LIMIT:
            raise ValueError(
                f"{channel_name}: pulse range [{min_pulse}, {max_pulse}] us exceeds hardware "
                f"limits [{self.PULSE_MIN_LIMIT}, {self.PULSE_MAX_LIMIT}] us.")

    def _init_servos(self, prop_min, prop_max, rudder_min, rudder_max) -> None:
        """Set up the three servo channels on the PCA9685 (ported from motor.py)."""
        self._validate_pulse_range(prop_min, prop_max, "prop_l (ch 0)")
        self._validate_pulse_range(prop_min, prop_max, "prop_r (ch 1)")
        self._validate_pulse_range(rudder_min, rudder_max, "rudder (ch 2)")

        try:
            self.prop_l = servo.Servo(self.pca.channels[0], min_pulse=prop_min, max_pulse=prop_max)
            self.prop_r = servo.Servo(self.pca.channels[1], min_pulse=prop_min, max_pulse=prop_max)
            self.rudder = servo.Servo(self.pca.channels[2], min_pulse=rudder_min, max_pulse=rudder_max)
        except Exception as e:
            self.get_logger().error(f"Failed to initialize servo channels: {e}")
            raise

        self.get_logger().info("Servo PWM channels initialized (ch0 prop_l, ch1 prop_r, ch2 rudder).")

    # ------------------------------------------------------------------ command handling
    def _apply(self, commands: dict) -> None:
        """Write a {joint_name: fraction} mapping to the servos and echo the state.

        Args:
            commands: fraction in [0, 1] per joint name. Values are clamped defensively.
        """
        applied = {}
        for name, fraction in commands.items():
            servo_obj = self.joint_to_servo.get(name)
            if servo_obj is None:
                self.get_logger().warning(f"Unknown joint '{name}' in command, ignoring.")
                continue
            fraction = max(0.0, min(1.0, float(fraction)))
            servo_obj.fraction = fraction
            applied[name] = fraction

        self._publish_state(applied)

    def command_callback(self, msg: JointState) -> None:
        """Handle an incoming JointState command from topic_based_ros2_control."""
        field = getattr(msg, self.JOINT_COMMAND_FIELD)  # msg.position by default
        if not msg.name or len(field) < len(msg.name):
            self.get_logger().warning(
                f"JointState '{self.JOINT_COMMAND_FIELD}' field shorter than name list, ignoring.")
            return

        commands = {name: field[i] for i, name in enumerate(msg.name)}
        self._apply(commands)
        self.last_cmd_time = self.get_clock().now()

    def watchdog(self) -> None:
        """If commands have gone stale, force neutral (dead-man switch)."""
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if age > self.cmd_timeout:
            self._apply(dict(self.joint_neutral))
            # NOTE: do not reset last_cmd_time here, so we keep holding neutral until a
            # real command resumes. This logs at most once per second to avoid spam.
            self.get_logger().warning(
                'No fresh /asv/joint_commands -> holding neutral.',
                throttle_duration_sec=1.0)

    def _publish_state(self, applied: dict) -> None:
        """Echo the last-written fractions as JointState (open-loop: no encoders on thrusters)."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(applied.keys())
        # State interface is declared as "position" in the URDF, so fill position with the
        # commanded fraction. There is no real feedback sensor -- this is an honest echo.
        msg.position = [applied[name] for name in msg.name]
        self.state_pub.publish(msg)

    def publish_battery(self) -> None:
        #Publish the current battery voltages
        battery_dwL = self.ina[0].bus_voltage
        battery_dwR = self.ina[1].bus_voltage
        battery_br = self.ina[2].bus_voltage

        msg = Float32MultiArray()
        msg.data = [battery_dwL, battery_dwR, battery_br]
        self.battery_pub.publish(msg)

    def shutdown(self) -> None:
        """Return channels to neutral and release the PCA9685 on node shutdown."""
        try:
            self._apply(dict(self.joint_neutral))
        finally:
            self.pca.deinit()


def main(args=None) -> None:
    """Initialize the ROS2 node and spin."""
    rclpy.init(args=args)
    node = Pca9685Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('pca9685_driver interrupted by user.')
    except Exception as e:
        node.get_logger().error(f'pca9685_driver error: {e}')
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
