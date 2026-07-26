"""Bench-test utility: drive forward in a straight line for a fixed duration.

    [THIS NODE] --/cmd_vel (linear.x only)--> thrust_mixer --> ... --> real thrusters

Standalone -- NOT part of bringup.launch.py or task1.launch.py. Run it in a second
terminal ALONGSIDE an already-running bringup (thrust_mixer must already be up,
subscribed to /cmd_vel); it does not start the ZED/SLAM/ros2_control stack itself.

Usage:
    ros2 launch loone forward.launch.py
    ros2 launch loone forward.launch.py linear_speed:=0.3 duration:=10.0
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class ForwardNode(Node):
    """Publish a constant forward-surge Twist on /cmd_vel for `duration` seconds, then stop."""

    def __init__(self) -> None:
        super().__init__('forward_node')

        self.declare_parameter('linear_speed', 0.3)   # m/s -- keep small, see thrust_mixer.py
        self.declare_parameter('duration', 10.0)      # seconds; <= 0 means drive forever
        self.declare_parameter('publish_rate', 10.0)  # Hz; keep > thrust_mixer's dead-man cmd_timeout

        self.linear_speed = self.get_parameter('linear_speed').value
        self.duration = self.get_parameter('duration').value
        publish_rate = self.get_parameter('publish_rate').value

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.start_time = self.get_clock().now()
        self._done = False
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_forward)

        self.get_logger().info(
            f'forward_node: driving forward at {self.linear_speed} m/s' +
            ('' if self.duration <= 0 else f' for {self.duration}s'))

    def publish_forward(self) -> None:
        """Timer callback: publish the forward command, or stop once `duration` elapses."""
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        if 0 < self.duration <= elapsed:
            self.cmd_pub.publish(Twist())  # zero Twist -- thrust_mixer reads this as stop
            self.get_logger().info('forward_node: duration elapsed, stopping.')
            self.timer.cancel()
            self._done = True
            return

        msg = Twist()
        msg.linear.x = self.linear_speed
        self.cmd_pub.publish(msg)


def main(args=None) -> None:
    """Initialize the ROS2 node and spin until `duration` elapses or interrupted."""
    rclpy.init(args=args)
    node = ForwardNode()
    try:
        while rclpy.ok() and not node._done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info('forward_node interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
