"""hardware_test: short one-shot hardware demo, then exits.

Prints battery levels, pulses both motors, exercises the ball loader/launcher
and flashes the LEDs. Stop the motor node before running (same I2C bus).
"""

import time

import board
import busio
import rclpy
from adafruit_ina3221 import INA3221
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
from rclpy.node import Node

from loone.battery import battery_voltage


class HardwareTest(Node):
    """Run the PCA9685 / INA3221 hardware demo once."""

    def __init__(self) -> None:
        super().__init__('hardware_test')

    def load(self, launcher, carriage) -> None:
        """Run the launcher and carriage in sequence to load a ball."""
        self.get_logger().info('Load ball, then press Enter to run the loader.')
        carriage.angle = 45
        time.sleep(1)
        input('')
        try:
            launcher.throttle = -0.5
            time.sleep(1)
            carriage.throttle = 0.5
            time.sleep(1)
        finally:
            launcher.throttle = 0.0
            carriage.throttle = 0.0

    def launch(self, launcher, carriage) -> None:
        """Run the launcher and carriage to fire a loaded ball."""
        self.get_logger().info('Press Enter to launch the ball.')
        input('')
        try:
            launcher.throttle = 0.5
            carriage.throttle = -0.5
            time.sleep(4)
        finally:
            launcher.throttle = 0.0
            carriage.throttle = 0.0

    def run(self) -> None:
        i2c = busio.I2C(board.SCL, board.SDA)
        pca = PCA9685(i2c, address=64)
        try:
            pca.frequency = 50
            left = servo.Servo(pca.channels[0], min_pulse=1120, max_pulse=1880)
            right = servo.Servo(pca.channels[1], min_pulse=1120, max_pulse=1880)
            carriage = servo.ContinuousServo(
                pca.channels[2], min_pulse=1220, max_pulse=1820)
            launcher = servo.ContinuousServo(
                pca.channels[3], min_pulse=1120, max_pulse=1880)
            red, green = pca.channels[4], pca.channels[5]
            left.fraction = right.fraction = 0.5
            carriage.throttle = 0.0
            launcher.throttle = 0.0
            ina = INA3221(i2c, address=65, enable=[0, 1, 2])

            try:
                for name, channel, low, high in (
                    ("Left", 0, 15.0, 21.0),
                    ("Right", 1, 15.0, 21.0),
                    ("Main", 2, 13.2, 16.8),
                ):
                    voltage = battery_voltage(ina[channel])
                    percent = max(0, min(100, round((voltage - low) / (high - low) * 100)))
                    self.get_logger().info(f"{name} battery: {percent}% ({voltage:.1f} V)")

                self.load(launcher, carriage)
                self.get_logger().info("Motors...")
                left.fraction = right.fraction = 0.56
                time.sleep(0.5)
                left.fraction = right.fraction = 0.5

                self.launch(launcher, carriage)
                self.load(launcher, carriage)

                self.get_logger().info("Party mode!")
                for _ in range(2):
                    for r, g in ((0xFFFF, 10), (0xFFFF, 0xFFFF), (0, 0xFFFF), (0, 0)):
                        red.duty_cycle, green.duty_cycle = r, g
                        time.sleep(0.15)
            finally:
                left.fraction = right.fraction = 0.5
                launcher.throttle = 0.0
                carriage.throttle = 0.0
                red.duty_cycle = green.duty_cycle = 0
        finally:
            pca.deinit()


def main(args=None) -> None:
    """Initialize the ROS2 node, run the demo once and exit."""
    rclpy.init(args=args)
    node = HardwareTest()
    try:
        node.run()
        node.get_logger().info('hardware_test done.')
    except KeyboardInterrupt:
        node.get_logger().info('hardware_test interrupted by user.')
    except Exception as e:
        node.get_logger().error(f'hardware_test error: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
