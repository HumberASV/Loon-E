import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import threading
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

    def __init__(self) -> None:
        """Initialize the Motor node, configure PCA9685, and set up servo PWM channels."""
        super().__init__('Motor_PubSub')

        # we need value from phone and task to drive the motors
        # check if we have received data from both before executing
        self.phone_data_ready_event = threading.Event()
        self.task_data_ready_event = threading.Event()

        # subscribe to phone and task topics, publish to motor topic
        self.phone_sub = self.create_subscription(Float32MultiArray, 'phone', self.phone_callback, 10)
        self.task_sub = self.create_subscription(Float32MultiArray, 'task', self.task_callback, 10)
        self.motor_pub = self.create_publisher(Float32MultiArray, 'motor', 10)

        # Declare parameters with fallback/default values
        self.declare_parameter('freq', 50)
        self.declare_parameter('factor', 0.75)
        self.declare_parameter('center', 0.55)
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)
        self.declare_parameter('max', 45.0)
        self.declare_parameter('prop_min', 1120)
        self.declare_parameter('prop_max', 1880)
        self.declare_parameter('rudder_min', 1220)
        self.declare_parameter('rudder_max', 1820)

        # Retrieve parameters
        freq       = self.get_parameter('freq').value
        self.factor = self.get_parameter('factor').value
        self.center = self.get_parameter('center').value
        self.kp    = self.get_parameter('kp').value
        self.ki    = self.get_parameter('ki').value
        self.kd    = self.get_parameter('kd').value
        self.max   = self.get_parameter('max').value
        prop_min   = self.get_parameter('prop_min').value
        prop_max   = self.get_parameter('prop_max').value
        rudder_min = self.get_parameter('rudder_min').value
        rudder_max = self.get_parameter('rudder_max').value

        self._init_pca(freq)
        self._init_servos(prop_min, prop_max, rudder_min, rudder_max)

        #Other internal variables
        self.i = 0
        self.last_error = 0
        self.last_time = time.time()

        #Other variables from topics
        self.current_speed = np.nan
        self.current_heading = np.nan
        self.target_heading = None
        self.target_speed = None

        # Spin until data is received
        self.get_logger().info('waiting for phone and task data...')
        while not (self.phone_data_ready_event.is_set()
                   and self.task_data_ready_event.is_set()):
            rclpy.spin_once(self, timeout_sec = 0.1)
        self.get_logger().info('phone and task data received, starting motor control loop.')

    def _init_pca(self, freq) -> None:
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

    def _validate_pulse_range(self, min_pulse, max_pulse, channel_name) -> None:
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
            
    def publish_motor(self) -> None:
        # Publish the current motor state
        msg = Float32MultiArray()
        msg.data = [self.prop_l.fraction, self.prop_r.fraction, self.rudder.fraction]
        self.motor_pub.publish(msg)
        #self.get_logger().info(f"Motor: {msg.data}")
        
    def _init_servos(self, prop_min, prop_max, rudder_min, rudder_max) -> None:
        """Set up servo PWM channels on the PCA9685 with validated pulse ranges.

        Raises:
            ValueError: If any pulse range is invalid (see _validate_pulse_range).
            Exception: If a servo channel cannot be acquired from the PCA9685.
        """
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

    def convert(self, angle) -> float:
        """Convert a heading angle from [0, 360] to [-180, 180].
        
        Args:
            angle: Heading in degrees
        
        Returns:
            Converted angle
        """
        if angle > 180:
            angle -= 360

        return angle

    def remap(self, error, outMin=1540, outMax=1880) -> float:
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

    def get_fraction(self, pulse, min_pulse=1120, max_pulse=1880) -> float:
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
    
    def reverse(self) -> None:
        """Set PWM to move backwards"""
        self.prop_l.fraction = 0 #max backward
        self.prop_r.fraction = 0 #max backward
        self.rudder.fraction = self.center #0 degrees
        self.publish_motor()
    
    def stop(self) -> None:
        """Set propeller and rudder PWM to center/no motion"""
        self.prop_l.fraction = 0.5 #no motion
        self.prop_r.fraction = 0.5 #no motion
        self.rudder.fraction = self.center #0 degrees
        self.publish_motor()

    def turn_in_place(self) -> None:
        """Set PWM to turn in place"""
        if np.sign(self.dir) == 1: #turn left
            self.prop_l.fraction = self.get_fraction(1460) #min backward
            self.prop_r.fraction = self.get_fraction(1540) #min forward 
            self.rudder.fraction = self.center #0 degrees
        
        else: #turn right
            self.prop_l.fraction = self.get_fraction(1540) #min forward
            self.prop_r.fraction = self.get_fraction(1460) #min backward 
            self.rudder.fraction = self.center #0 degrees
        
        self.publish_motor()

    def drive(self) -> None:
        """Run one PID control cycle and update propeller and rudder PWM outputs."""
        current_time = time.time()
        
        #defensive check to ensure we have valid target and current heading/speed values
        if (np.isnan(self.current_heading) or np.isnan(self.current_speed)
                or self.target_heading is None or self.target_speed is None):
            self.get_logger().warning("Sensor/target data not ready; skipping drive cycle.")
            return
        
        current_error = self.target_heading - self.current_heading
        dt = current_time - self.last_time
        de = (current_error - self.last_error) / dt

        if current_error <= 45:
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
                self.rudder.fraction = 0.0    # 35° right
            elif output > self.max / 2:
                self.rudder.fraction = 1.0    # 35° left
            else:
                self.rudder.fraction = self.center  # centred
                
            self.last_error = current_error
            self.last_time = current_time
            self.publish_motor()
        
        else:
            self.dir = current_error
            self.turn_in_place()
    
    def check_data(self) -> None:
        """Executes action based on value of self.command"""
        match self.command:
            case -1: #reverse
                self.reverse()

            case 0: #stop
                self.stop()

            case 1: #drive
                self.drive()
            
            case 2: #turn
                if not np.isnan(self.dir):
                    self.turn_in_place()

    def phone_callback(self, msg) -> None:  
        """Handle incoming phone telemetry and update current position, speed, and heading.

        Args:
            msg: Float32MultiArray where
                index 0 is latitude
                index 1 is longitude
                index 2 is speed
                index 3 is heading.
        """
        data = msg.data
        self.get_logger().info(f"Phone: {msg.data}")
        self.current_speed = data[2]
        self.current_heading = data[3]
        self.phone_data_ready_event.set() # Unblocks the init sequence

    def task_callback(self, msg) -> None:
        """Handle incoming task commands and drive motors if sensor data is ready.

        Args:
            msg: Float32MultiArray where index 1 is target heading and index 2 is target speed.
        """
        data = msg.data
        self.get_logger().info(f"Task: {msg.data}")
        self.command = data[0]
        self.target_heading = data[1]
        self.target_speed = data[2]
        self.dir = data[3]
        self.check_data()
        self.task_data_ready_event.set() # Unblocks the init sequence

    def shutdown(self) -> None:
        """De-initialize the PCA9685 and release the I2C bus on node shutdown."""
        self.pca.deinit()


def main(args=None) -> None:
    """ Main function to initialize the ROS2 node and start spinning. """
    rclpy.init(args=args)
    motor = Motor()
    try:
        rclpy.spin(motor)
    except KeyboardInterrupt:
        motor.get_logger().info("Motor node interrupted by user.")
    except Exception as e:
        motor.get_logger().error(f"Motor node encountered an error: {e}")
    finally:
        motor.shutdown()
        motor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
