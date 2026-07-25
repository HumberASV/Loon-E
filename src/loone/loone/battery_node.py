"""battery_node: raw INA3221 voltages -> proper sensor_msgs/BatteryState.

    busio_node (the only node touching I2C / the INA3221)
        --> battery_raw (std_msgs/Float32MultiArray: [dwL, dwR, br] volts)
        --> [THIS NODE] volts -> percentage / health per battery
        --> battery_status/prop_l  (sensor_msgs/BatteryState, Dewalt, left propulsion)
        --> battery_status/prop_r  (sensor_msgs/BatteryState, Dewalt, right propulsion)
        --> battery_status/main    (sensor_msgs/BatteryState, Blue Robotics, primary battery)

Percentage/health thresholds mirror config.yaml's /motor block and motor.py's
convert_battery(): dw_min/dw_max for the two Dewalt propulsion batteries,
br_min/br_max for the Blue Robotics primary battery.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32MultiArray


class BatteryNode(Node):
    """Convert raw INA3221 bus voltages into per-battery BatteryState messages."""

    def __init__(self) -> None:
        super().__init__('battery_node')

        # ---- Parameters (defaults match config.yaml /motor block) ----
        self.declare_parameter('dw_min', 15.0)   # Dewalt (prop) battery min voltage
        self.declare_parameter('dw_max', 21.0)   # Dewalt (prop) battery max voltage
        self.declare_parameter('br_min', 13.2)   # Blue Robotics (main) battery min voltage
        self.declare_parameter('br_max', 16.8)   # Blue Robotics (main) battery max voltage

        self.dw_min = self.get_parameter('dw_min').value
        self.dw_max = self.get_parameter('dw_max').value
        self.br_min = self.get_parameter('br_min').value
        self.br_max = self.get_parameter('br_max').value

        self.battery_raw_sub = self.create_subscription(
            Float32MultiArray, 'battery_raw', self.battery_raw_callback, 10)
        self.prop_l_pub = self.create_publisher(BatteryState, 'battery_status/prop_l', 10)
        self.prop_r_pub = self.create_publisher(BatteryState, 'battery_status/prop_r', 10)
        self.main_pub = self.create_publisher(BatteryState, 'battery_status/main', 10)

        self.get_logger().info('battery_node ready, waiting for battery_raw.')

    def _to_battery_state(
            self, voltage: float, min_v: float, max_v: float, location: str) -> BatteryState:
        """Build a BatteryState from a raw voltage and the battery's nominal range."""
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = float(voltage)
        msg.percentage = max(0.0, min(1.0, (voltage - min_v) / (max_v - min_v)))
        msg.present = True
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        if voltage < min_v:
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_DEAD
        elif voltage > max_v:
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_OVERVOLTAGE
        else:
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        msg.location = location
        return msg

    def battery_raw_callback(self, msg: Float32MultiArray) -> None:
        """Handle a [dwL, dwR, br] raw voltage reading from busio_node."""
        if len(msg.data) < 3:
            self.get_logger().warning(
                f"battery_raw expected 3 values, got {len(msg.data)}, ignoring.")
            return

        dw_l, dw_r, br = msg.data[0], msg.data[1], msg.data[2]
        self.prop_l_pub.publish(self._to_battery_state(dw_l, self.dw_min, self.dw_max, 'prop_l'))
        self.prop_r_pub.publish(self._to_battery_state(dw_r, self.dw_min, self.dw_max, 'prop_r'))
        self.main_pub.publish(self._to_battery_state(br, self.br_min, self.br_max, 'main'))


def main(args=None) -> None:
    """Initialize the ROS2 node and spin."""
    rclpy.init(args=args)
    node = BatteryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('battery_node interrupted by user.')
    except Exception as e:
        node.get_logger().error(f'battery_node error: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
