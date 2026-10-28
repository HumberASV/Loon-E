"""hardware_test: short one-shot hardware demo, then exits.

Prints battery levels, pulses both motors, sweeps the rudder and flashes the
LEDs. Stop the motor node before running (this uses the same I2C bus).
"""

import time

import board
import busio
import rclpy
from adafruit_ina3221 import INA3221
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
from rclpy.node import Node


class HardwareTest(Node):
    """Run the PCA9685 / INA3221 hardware demo once."""

    def __init__(self) -> None:
        super().__init__('hardware_test')

    def run(self) -> None:
        i2c = busio.I2C(board.SCL, board.SDA)
        pca = PCA9685(i2c, address=64)
        try:
            pca.frequency = 50
            left = servo.Servo(pca.channels[0], min_pulse=1120, max_pulse=1880)
            right = servo.Servo(pca.channels[1], min_pulse=1120, max_pulse=1880)
            rudder = servo.Servo(pca.channels[2], min_pulse=1220, max_pulse=1820)
            red, green = pca.channels[4], pca.channels[5]
            left.fraction = right.fraction = 0.5
            rudder.angle = 90
            ina = INA3221(i2c, address=65, enable=[0, 1, 2])

            try:
                for name, voltage, low, high in (
                    ("Left", ina[0].bus_voltage, 15.0, 21.0),
                    ("Right", ina[1].bus_voltage, 15.0, 21.0),
                    ("Main", ina[2].bus_voltage, 13.2, 16.8),
                ):
                    percent = max(0, min(100, round((voltage - low) / (high - low) * 100)))
                    self.get_logger().info(f"{name} battery: {percent}% ({voltage:.1f} V)")

                self.get_logger().info("Motors...")
                left.fraction = right.fraction = 0.56
                time.sleep(0.5)
                left.fraction = right.fraction = 0.5

                self.get_logger().info("Rudder...")
                for angle in (60, 120, 90):
                    rudder.angle = angle
                    time.sleep(0.3)

                self.get_logger().info("Party mode!")
                for _ in range(2):
                    for r, g in ((0xFFFF, 10), (0xFFFF, 0xFFFF), (0, 0xFFFF), (0, 0)):
                        red.duty_cycle, green.duty_cycle = r, g
                        time.sleep(0.15)
            finally:
                left.fraction = right.fraction = 0.5
                rudder.angle = 90
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
