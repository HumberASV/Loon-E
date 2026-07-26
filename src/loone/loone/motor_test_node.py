"""Bench-test utility: toggle the thrusters on and off in a simple duty cycle.

    [THIS NODE] --/cmd_vel (linear.x only)--> thrust_mixer --> ... --> real thrusters

Standalone actuator sanity check -- does NOT touch nav2/SLAM/the ZED camera. Meant
to run under motor_test.launch.py, which brings up only the control chain
(controller_manager + thrust_mixer + busio_node/sim_state_echo), so you can confirm
the props/rudder actually respond before trusting the full nav2 stack.

Usage:
    ros2 launch loone motor_test.launch.py
    ros2 launch loone motor_test.launch.py thrust:=0.1 on_duration:=2.0 off_duration:=2.0 cycles:=5
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MotorTestNode(Node):
    """Alternate a fixed forward-thrust Twist with a zero (neutral) Twist, `cycles` times."""

    def __init__(self) -> None:
        super().__init__('motor_test_node')

        self.declare_parameter('thrust', 0.1)        # linear.x fraction fed to thrust_mixer
        self.declare_parameter('on_duration', 2.0)   # seconds thrust is applied per cycle
        self.declare_parameter('off_duration', 2.0)  # seconds held neutral between cycles
        self.declare_parameter('cycles', 3)          # number of on/off cycles; <= 0 forever

        self.thrust = self.get_parameter('thrust').value
        self.on_duration = self.get_parameter('on_duration').value
        self.off_duration = self.get_parameter('off_duration').value
        self.cycles = self.get_parameter('cycles').value

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.start_time = self.get_clock().now()
        self._done = False
        # 10 Hz keeps every phase above thrust_mixer's dead-man cmd_timeout (default 0.5s).
        self.timer = self.create_timer(0.1, self.publish_toggle)

        self.get_logger().info(
            f'motor_test_node: thrust={self.thrust} on={self.on_duration}s '
            f'off={self.off_duration}s ' +
            ('forever' if self.cycles <= 0 else f'for {self.cycles} cycle(s)'))

    def publish_toggle(self) -> None:
        """Timer callback: publish thrust or neutral depending on where we are in the cycle."""
        period = self.on_duration + self.off_duration
        total_elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

        if self.cycles > 0 and total_elapsed >= self.cycles * period:
            self.cmd_pub.publish(Twist())
            self.get_logger().info('motor_test_node: cycles complete, stopping.')
            self.timer.cancel()
            self._done = True
            return

        phase_elapsed = total_elapsed % period
        msg = Twist()
        msg.linear.x = self.thrust if phase_elapsed < self.on_duration else 0.0
        self.cmd_pub.publish(msg)


def main(args=None) -> None:
    """Initialize the ROS2 node and spin until `cycles` complete or interrupted."""
    rclpy.init(args=args)
    node = MotorTestNode()
    try:
        while rclpy.ok() and not node._done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info('motor_test_node interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
