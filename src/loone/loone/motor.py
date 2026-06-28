import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import time
import busio
import board
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685


class Motor(Node):
    """ROS2 node that controls motor outputs via a PCA9685 PWM driver.

    Subscribes to Phone telemetry (current speed/heading) and Task commands
    (target heading/speed), then runs a PID loop to drive two propellers and
    a rudder servo via the PCA9685.
    """

    # Valid frequency range for PCA9685 (Hz)
    PCA_FREQ_MIN = 24
    PCA_FREQ_MAX = 1526

    # Absolute pulse width limits enforced by hardware (microseconds)
    PULSE_MIN_LIMIT = 500
    PULSE_MAX_LIMIT = 2500

    def __init__(self):
        """Initialize the Motor node, configure PCA9685, and set up servo PWM channels."""
        super().__init__('Motor_Sub')
        self.phone_sub = self.create_subscription(Float32MultiArray, 'phone', self.phone_callback, 10)
        self.publisher_ = self.create_publisher(Float32MultiArray, 'motor', 10)
        self.task_sub = self.create_subscription(Float32MultiArray, 'task', self.task_callback, 10)

        freq = 50
        self._init_pca(freq)
        self._init_servos()

        self.factor = 0.75

        self.kp = 1
        self.ki = 0
        self.kd = 0
        self.i = 0
        self.last_error = 0
        self.last_time = time.time()
        self.max = 45

        self.current_speed = np.nan
        self.current_heading = np.nan
        self.target_heading = None
        self.target_speed = None

    def _init_pca(self, freq):
        """Initialize the PCA9685 PWM driver over I2C.

        Args:
            freq: PWM frequency in Hz. Must be within [PCA_FREQ_MIN, PCA_FREQ_MAX].

        Raises:
            ValueError: If freq is outside the supported range.
            Exception: If the I2C bus or PCA9685 device cannot be reached.
        """
        if not (self.PCA_FREQ_MIN <= freq <= self.PCA_FREQ_MAX):
            self.get_logger().error(
                f"PCA9685 frequency {freq} Hz is out of valid range "
                f"[{self.PCA_FREQ_MIN}, {self.PCA_FREQ_MAX}] Hz."
            )
            raise ValueError(f"Invalid PCA9685 frequency: {freq}")

        try:
            i2c = busio.I2C(board.SCL, board.SDA)
        except Exception as e:
            self.get_logger().error(
                f"Failed to initialize I2C bus. Check that SDA/SCL pins are correct "
                f"and the bus is enabled: {e}"
            )
            raise

        try:
            self.pca = PCA9685(i2c)
        except Exception as e:
            self.get_logger().error(
                f"PCA9685 not found on the I2C bus. Verify wiring, pull-up resistors, "
                f"and the I2C address (default 0x40): {e}"
            )
            raise

        self.pca.frequency = freq
        self.pulse = 1 / freq * 10**6
        self.get_logger().info(f"PCA9685 initialized at {freq} Hz.")

    def _validate_pulse_range(self, min_pulse, max_pulse, channel_name):
        """Validate that PWM pulse widths are ordered and within hardware limits.

        Args:
            min_pulse: Minimum pulse width in microseconds.
            max_pulse: Maximum pulse width in microseconds.
            channel_name: Human-readable name used in error messages.

        Raises:
            ValueError: If min_pulse >= max_pulse or either value falls outside
                        [PULSE_MIN_LIMIT, PULSE_MAX_LIMIT].
        """
        if min_pulse >= max_pulse:
            self.get_logger().error(
                f"{channel_name}: min_pulse ({min_pulse} µs) must be less than "
                f"max_pulse ({max_pulse} µs)."
            )
            raise ValueError(f"Invalid pulse range for {channel_name}: [{min_pulse}, {max_pulse}]")

        if min_pulse < self.PULSE_MIN_LIMIT or max_pulse > self.PULSE_MAX_LIMIT:
            self.get_logger().error(
                f"{channel_name}: pulse range [{min_pulse}, {max_pulse}] µs exceeds "
                f"hardware limits [{self.PULSE_MIN_LIMIT}, {self.PULSE_MAX_LIMIT}] µs."
            )
            raise ValueError(f"Pulse range out of hardware limits for {channel_name}")

    def _init_servos(self):
        """Set up servo PWM channels on the PCA9685 with validated pulse ranges.

        Raises:
            ValueError: If any pulse range is invalid (see _validate_pulse_range).
            Exception: If a servo channel cannot be acquired from the PCA9685.
        """
        prop_min, prop_max = 1120, 1880
        rudder_min, rudder_max = 1220, 1820

        self._validate_pulse_range(prop_min, prop_max, "prop_l (ch 0)")
        self._validate_pulse_range(prop_min, prop_max, "prop_r (ch 1)")
        self._validate_pulse_range(rudder_min, rudder_max, "rudder (ch 2)")

        try:
            self.prop_l = servo.Servo(self.pca.channels[0], min_pulse=prop_min, max_pulse=prop_max)
            self.prop_r = servo.Servo(self.pca.channels[1], min_pulse=prop_min, max_pulse=prop_max)
            self.rudder = servo.Servo(self.pca.channels[2], min_pulse=rudder_min, max_pulse=rudder_max)
        except Exception as e:
            self.get_logger().error(
                f"Failed to initialize servo PWM channels on PCA9685: {e}"
            )
            raise

        self.get_logger().info("Servo PWM channels initialized.")

    def convert(self, angle):
        """Convert a heading angle from [0, 360] to [-180, 180].
        
        Args:
            angle: Heading in degrees
        
        Returns:
            Converted angle
        """
        if angle > 180:
            angle -= 360

        return angle

    def remap(self, error, outMin=1540, outMax=1880):
        """Map a heading error to a proportional pulse width in microseconds.

        Larger errors produce lower pulse widths (stronger correction).

        Args:
            error: Heading error in degrees.
            outMin: Pulse width (µs) corresponding to maximum correction.
            outMax: Pulse width (µs) corresponding to minimum correction.

        Returns:
            Pulse width in microseconds.
        """
        output = outMax + (abs(error) / self.max * (outMin - outMax))
        return output

    def get_fraction(self, pulse, min_pulse=1120, max_pulse=1880):
        """Convert a pulse width in microseconds to a normalized duty cycle fraction.

        Args:
            pulse: Pulse width in microseconds.
            min_pulse: Pulse width that maps to fraction 0.0.
            max_pulse: Pulse width that maps to fraction 1.0.

        Returns:
            Duty cycle fraction clamped to [0.0, 1.0].

        Raises:
            ValueError: If min_pulse >= max_pulse.
        """
        if min_pulse >= max_pulse:
            self.get_logger().error(
                f"get_fraction: min_pulse ({min_pulse} µs) must be less than "
                f"max_pulse ({max_pulse} µs)."
            )
            raise ValueError("min_pulse must be less than max_pulse in get_fraction")

        fraction = (pulse - min_pulse) / (max_pulse - min_pulse)

        if not (0.0 <= fraction <= 1.0):
            self.get_logger().warning(
                f"get_fraction: pulse {pulse} µs yields fraction {fraction:.3f} "
                f"outside [0.0, 1.0] — clamping."
            )
            fraction = max(0.0, min(1.0, fraction))

        return fraction

    def drive(self):
        """Run one PID control cycle and update propeller and rudder PWM outputs."""
        current_time = time.time()
        current_error = self.target_heading - self.current_heading
        dt = current_time - self.last_time
        de = (current_error - self.last_error) / dt

        self.i = self.i + self.ki * current_error
        if self.i < -self.max:
            self.i = -self.max
        elif self.i > self.max:
            self.i = self.max

        output = self.kp * current_error + self.i * dt + self.kd * de
        if output < -self.max:
            output = -self.max
        elif output > self.max:
            output = self.max

        # This remapping ensures that the output pulse width 
        # is within the valid range for the propellers ?
        remapped_output = self.remap(output)

        if self.current_speed < self.target_speed:
            self.factor += 0.05
            if self.factor > 1:
                self.factor = 1
        elif self.current_speed > self.target_speed:
            self.factor -= 0.05
            if self.factor < 0.55:
                self.factor = 0.55

        if current_error > 0:  # turn right: reduce right propeller
            self.prop_l.fraction = self.factor
            self.prop_r.fraction = self.get_fraction(remapped_output) * self.factor
            self.get_logger().info(f"Sending Right: {remapped_output}")
        else:  # turn left: reduce left propeller
            self.prop_l.fraction = self.get_fraction(remapped_output) * self.factor
            self.prop_r.fraction = self.factor
            self.get_logger().info(f"Sending Left: {remapped_output}")

        if output < -self.max / 2:
            self.rudder.fraction = 0    # 35° right
        elif output > self.max / 2:
            self.rudder.fraction = 1    # 35° left
        else:
            self.rudder.fraction = 0.55  # centred

        # Publish the current motor state as a Float32MultiArray
        self.publisher_.publish(Float32MultiArray(data=[self.prop_l.fraction, self.prop_r.fraction, self.rudder.fraction]))
        self.last_error = current_error
        self.last_time = current_time

    def check_data(self):
        """Return True if all required sensor and target data are available and valid."""
        return not (
            np.isnan(self.current_heading)
            or np.isnan(self.current_speed)
            or self.target_heading is None
            or self.target_speed is None
        )

    def phone_callback(self, msg):
        """Handle incoming phone telemetry and update current speed and heading.

        Args:
            msg: Float32MultiArray where index 2 is speed and index 3 is heading.
        """
        data = msg.data
        self.get_logger().info(f"Phone: {msg.data}")
        self.current_speed = data[2]
        self.current_heading = data[3]

    def task_callback(self, msg):
        """Handle incoming task commands and drive motors if sensor data is ready.

        Args:
            msg: Float32MultiArray where index 1 is target heading and index 2 is target speed.
        """
        data = msg.data
        self.get_logger().info(f"Task: {msg.data}")
        self.target_heading = data[1]
        self.target_speed = data[2]
        if self.check_data():
            self.drive()

    def motor_callback(self, msg):
        """Publish current motor state as a Float32MultiArray.

        Args:
            msg: Float32MultiArray containing motor state data.
        """
        self.publisher_.publish(msg)

    def shutdown(self):
        """De-initialize the PCA9685 and release the I2C bus on node shutdown."""
        self.pca.deinit()


def main(args=None):
    rclpy.init(args=args)
    motor = Motor()

    # Add a try-except block to handle KeyboardInterrupt gracefully
    try:
        rclpy.spin(motor)
    except KeyboardInterrupt:
        motor.get_logger().info("Motor node interrupted by user.")
    finally:
        motor.shutdown()
        motor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
