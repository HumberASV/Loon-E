"""busio_node: JointState commands -> servo PWM over I2C (formerly pca9685_driver.py).

This node is the bottom of the chained-controls stack. It is the ONLY node that
touches the I2C bus / PCA9685 / INA3221 hardware:

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

This node also owns the INA3221 (same I2C bus, address 0x41, channels 0/1/2 wired
to the dwL/dwR/br batteries -- see config.yaml's dw_min/dw_max/br_min/br_max). It
only publishes the raw bus voltages on 'battery_raw'; battery_node.py subscribes
to that and converts it into proper sensor_msgs/BatteryState messages.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int8MultiArray, ColorRGBA
import threading
import numpy as np
import time
import busio
import board
import digitalio
from adafruit_ina3221 import INA3221
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

class MotorNode(Node):
    """Drive two propeller ESCs and one rudder servo on a PCA9685 from JointState commands."""

    # Valid frequency range for PCA9685 (Hz) -- copied from motor.py.
    PCA_FREQ_MIN = 24
    PCA_FREQ_MAX = 1526

    # Absolute pulse width limits enforced by hardware (microseconds) -- copied from motor.py.
    PULSE_MIN_LIMIT = 500
    PULSE_MAX_LIMIT = 2500

    def __init__(self) -> None:
        super().__init__('motor_node')

        # we need value from phone and task to drive the motors
        # check if we have received data from both before executing
        self.phone_data_ready_event = threading.Event()
        self.task_data_ready_event = threading.Event()

        # ---- Parameters (defaults mirror config.yaml /motor block and motor.py) ----
        self.declare_parameter('timer_period', 0.5) # Timer period to publish raw battery voltages
        self.declare_parameter('freq', 50)          # PCA9685 PWM frequency (Hz)
        self.declare_parameter('factor', 0.75)      # Starting speed
        self.declare_parameter('prop_min', 1120)    # propeller servo min pulse (us)
        self.declare_parameter('prop_max', 1880)    # propeller servo max pulse (us)
        self.declare_parameter('rudder_min', 1220)  # rudder servo min pulse (us)
        self.declare_parameter('rudder_max', 1820)  # rudder servo max pulse (us)
        self.declare_parameter('center', 0.5)   # propeller fraction = no thrust
        self.declare_parameter('kp', 2.0)
        self.declare_parameter('ki', 0.5)
        self.declare_parameter('kd', 0.02)
        self.declare_parameter('max', 45.0)
        self.declare_parameter('diff', 0.2)
        
        timer_period = self.get_parameter('timer_period').value
        freq = self.get_parameter('freq').value
        prop_min = self.get_parameter('prop_min').value
        prop_max = self.get_parameter('prop_max').value
        rudder_min = self.get_parameter('rudder_min').value
        rudder_max = self.get_parameter('rudder_max').value
        self.factor = self.get_parameter('factor').value
        self.center = self.get_parameter('center').value
        self.kp    = self.get_parameter('kp').value
        self.ki    = self.get_parameter('ki').value
        self.kd    = self.get_parameter('kd').value
        self.max   = self.get_parameter('max').value
        self.diff = self.get_parameter('diff').value
        
        # ---- Hardware bring-up (same sequence as motor.py) ----
        self._init_busio(freq)
        self._init_servos(prop_min, prop_max, rudder_min, rudder_max)
        self._init_led()

        self.phone_sub = self.create_subscription(Float32MultiArray, 'phone', self.phone_callback, 10)
        self.task_sub = self.create_subscription(Float32MultiArray, 'motor_command', self.task_callback, 10)
        self.motor_pub = self.create_publisher(Float32MultiArray, 'motor', 10)
        self.led_pub = self.create_publisher(ColorRGBA, 'led',10)
        # Raw INA3221 bus voltages for battery_node to convert into BatteryState messages.
        self.battery_raw_pub = self.create_publisher(Float32MultiArray, 'battery_raw', 10)
        # Raw autonomy / e-stop line levels. This node already owns board.D12 and
        # D13 for the status LEDs, and two processes opening the same gpiochip
        # line fails with "Device or resource busy" under libgpiod, so task.py
        # reads the switches from here instead of opening them a second time.
        # task.py fails closed if this goes stale, so publish at a steady rate.
        self.rc_state_pub = self.create_publisher(Int8MultiArray, 'rc_state', 10)
        self.battery_timer = self.create_timer(timer_period, self.publish_battery_raw)
        self.rc_state_timer = self.create_timer(timer_period, self.publish_rc_state)
        self.led_state_timer = self.create_timer(timer_period, self.publish_led)
        self.led_timer = self.create_timer(timer_period, self.set_led)
        self.get_logger().info('busio_node ready.')

        #Other internal variables
        self.i = 0
        self.last_error = 0
        self.last_time = time.time()

        # LED stuff
        self.led = ColorRGBA()
        self.led.r = 0.0
        self.led.g = 0.0
        self.led.b = 0.0
        self.led.a = 0.0

        #Other variables from topics
        self.current_speed = np.nan
        self.current_heading = np.nan
        self.target_heading = np.nan
        self.target_speed = np.nan
        self.dir = np.nan
        self.led_state = True

        # Spin until data is received
        self.get_logger().info('waiting for phone and task data...')
        while not (self.phone_data_ready_event.is_set()
                   and self.task_data_ready_event.is_set()):
            rclpy.spin_once(self, timeout_sec = 0.1)
        self.get_logger().info('phone and task data received, starting motor control loop.')

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

        self.pca.frequency = freq
        self.get_logger().info(f"PCA9685 initialized at {freq} Hz.")

        # INA3221 (battery voltage monitor) is a separate chip from the PCA9685 that
        # drives the propellers/rudder. A wiring fault on this sensor must not take
        # the motors down with it, so its failure is logged but not fatal here --
        # publish_battery_raw() skips publishing while self.ina is None.
        try:
            self.ina = INA3221(i2c, address = 65, enable = [0, 1, 2])
        except Exception as e:
            self.get_logger().error(
                f"INA3221 not found on I2C bus (check wiring/address 0x41): {e}. "
                "Battery telemetry disabled; motors are unaffected.")
            self.ina = None

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
            #self.rudder = servo.Servo(self.pca.channels[2], min_pulse=rudder_min, max_pulse=rudder_max)
            self.red = self.pca.channels[4]
            self.green = self.pca.channels[5]
        except Exception as e:
            self.get_logger().error(f"Failed to initialize servo channels: {e}")
            raise

        self.get_logger().info("Servo PWM channels initialized (ch0 prop_l, ch1 prop_r, ch2 rudder).")

    def _init_led(self):
        self.auto_pin = digitalio.DigitalInOut(board.D12)
        self.auto_pin.direction = digitalio.Direction.INPUT
        self.estop_pin = digitalio.DigitalInOut(board.D13)
        self.estop_pin.direction = digitalio.Direction.INPUT
        
    def publish_battery_raw(self) -> None:
        """Publish the three INA3221 bus voltages [dwL, dwR, br], unconverted.

        battery_node.py subscribes to this and turns it into proper
        sensor_msgs/BatteryState messages (percentage, health, per-battery topics).

        No-op if the INA3221 failed to initialize (see _init_busio).
        """
        if self.ina is None:
            return
        msg = Float32MultiArray()
        msg.data = [
            self.ina[0].bus_voltage,
            self.ina[1].bus_voltage,
            self.ina[2].bus_voltage,
        ]
        self.battery_raw_pub.publish(msg)

    def publish_rc_state(self) -> None:
        """Publish the raw autonomy / e-stop line levels as [auto, estop].

        The levels are published raw; task.py maps them through its
        auto_active_high / estop_clear_high parameters so the switch sense can
        be corrected from YAML without touching this node.
        """
        msg = Int8MultiArray()
        msg.data = [int(self.auto_pin.value), int(self.estop_pin.value)]
        self.rc_state_pub.publish(msg)

    def set_led(self):
        match self.led_state:
            case True:
                # yellow?
                self.red.duty_cycle = 0x7FFF
                self.led.r = 1.0 # should be equal to 0x7FFF
                # TODO(CARSON): probbably should make a conversion function or somthing
                self.green.duty_cycle = 0x7FFF
                self.led.g = 1.0
                

                if not self.estop_pin.value:
                    # red
                    self.green.duty_cycle = 0x0000
                    self.led.g = 1.0
                elif self.auto_pin.value:
                    # green
                    self.red.duty_cycle = 0x0000
                    self.led.r = 1.0

                self.led_state = False

            case False:
                # yellow
                self.red.duty_cycle = 0x0000
                self.led.r = 0.0
                self.green.duty_cycle = 0x0000
                self.led.g = 0.0
                self.led_state = True

    def publish_led(self) -> None:
        """Publish the current LED state"""
        msg = ColorRGBA()
        msg.r = self.led.r
        msg.g = self.led.g
        msg.b = self.led.b
        msg.a = 1.0
        self.led_pub.publish(msg)
        #self.get_logger().info(f"LED: {msg.r}, {msg.g}, {msg.b}, {msg.a}")

    def publish_motor(self) -> None:
        # Publish the current motor state
        msg = Float32MultiArray()
        msg.data = [
            self.prop_l.fraction,
            self.prop_r.fraction,
            # The rudder servo is disconnected (see the commented-out channel 2 in
            # _init_servos), so there is no real fraction to report. Publish the neutral
            # placeholder to keep the [left, right, rudder] wire format stable, and swap
            # this back to `self.rudder.fraction` when the hardware is repaired.
            0.5,
        ]
        self.motor_pub.publish(msg)
        self.get_logger().info(f"Motor: {msg.data}")

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
        self.prop_l.fraction = 0.4 # backward
        self.prop_r.fraction = 0.4 # backward
        #self.rudder.fraction = self.center #0 degrees
        self.publish_motor()
    
    def stop(self) -> None:
        """Set propeller and rudder PWM to center/no motion"""
        self.prop_l.fraction = 0.5 #no motion
        self.prop_r.fraction = 0.5 #no motion
        #self.rudder.fraction = self.center #0 degrees
        self.publish_motor()

    def turn_in_place(self) -> None:
        """Set PWM to turn in place"""
        if self.dir == 1: #turn left
            self.prop_l.fraction = self.center - self.diff # backward
            self.prop_r.fraction = self.center + self.diff # forward 
            #self.rudder.fraction = self.center #0 degrees
        
        else: #turn right
            self.prop_l.fraction = self.center + self.diff # forward
            self.prop_r.fraction = self.center - self.diff # backward 
            #self.rudder.fraction = self.center #0 degrees
        
        self.publish_motor()

    def drive(self) -> None:
        """Run one PID control cycle and update propeller and rudder PWM outputs."""
        current_time = time.time()        
        current_error = self.convert(self.convert(self.current_heading) - self.target_heading)
        dt = current_time - self.last_time
        # Two commands arriving inside the same clock tick gave dt = 0 and a
        # ZeroDivisionError in the middle of the control loop.
        if dt <= 0.0:
            return
        de = (current_error - self.last_error) / dt
        self.get_logger().info(f"{current_error}")
        if abs(current_error) <= 45:
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

            if current_error > 0:  # turn right: reduce right propeller
                self.prop_l.fraction = self.factor
                self.prop_r.fraction = self.get_fraction(remapped_output) * self.factor
                self.get_logger().info(f"Sending Right: {remapped_output}")
            else:  # turn left: reduce left propeller
                self.prop_l.fraction = self.get_fraction(remapped_output) * self.factor
                self.prop_r.fraction = self.factor
                self.get_logger().info(f"Sending Left: {remapped_output}")
            '''
            if output < -self.max / 2:
                self.rudder.fraction = 0.0    # 35° right
            elif output > self.max / 2:
                self.rudder.fraction = 1.0    # 35° left
            else:
                self.rudder.fraction = self.center  # centred
            '''    
            self.last_error = current_error
            self.last_time = current_time
            self.publish_motor()
        
        else:
            self.dir = np.sign(current_error)
            self.turn_in_place()
            self.get_logger().info("Turning in place")
    
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
                if self.dir != -999:
                    self.turn_in_place()

            case _: #other
                self.get_logger().info("Improper command, no action")

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
        self.dir = data[2]
        self.check_data()
        self.task_data_ready_event.set() # Unblocks the init sequence

    def shutdown(self) -> None:
        """Return channels to neutral and release the PCA9685 on node shutdown."""
        # _apply() and joint_neutral do not exist on this class -- the old body
        # raised AttributeError on every shutdown, so the propellers were never
        # commanded to neutral before deinit. stop() is the real neutral call.
        try:
            self.stop()
        except Exception as e:
            self.get_logger().error(f"Failed to neutralize motors on shutdown: {e}")
        finally:
            self.pca.deinit()

def main(args=None) -> None:
    """Initialize the ROS2 node and spin."""
    rclpy.init(args=args)
    node = MotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('motor_node interrupted by user.')
    except Exception as e:
        node.get_logger().error(f'motor_node error: {e}')
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
