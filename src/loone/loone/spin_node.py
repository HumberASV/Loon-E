"""Bench-test utility: spin the boat in place to pre-build SLAM Toolbox's map.

    [THIS NODE] --/cmd_vel (angular.z only)--> thrust_mixer --> ... --> real thrusters

Standalone -- NOT part of bringup.launch.py or task1.launch.py. Run it in a second
terminal ALONGSIDE an already-running bringup (thrust_mixer must already be up,
subscribed to /cmd_vel); it does not start the ZED/SLAM/ros2_control stack itself.

Why this exists: the ZED's depth camera only ever sees whatever is currently in
front of it, so a stationary boat leaves SLAM Toolbox's map (and therefore nav2's
global_costmap static layer) sized to a narrow forward wedge -- which can put the
robot's own origin outside the map bounds ("Robot is out of bounds of the costmap!").
Spinning in place sweeps the camera through a full circle so the map fills in
around the boat before a mission starts.

Usage:
    ros2 launch loone spin.launch.py
    ros2 launch loone spin.launch.py angular_speed:=0.3 duration:=20.0
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class SpinNode(Node):
    """Publish a constant yaw-rate Twist on /cmd_vel for `duration` seconds, then stop."""

    def __init__(self) -> None:
        super().__init__('spin_node')

        self.declare_parameter('angular_speed', 0.3)  # rad/s -- keep small, see thrust_mixer.py
        self.declare_parameter('duration', 20.0)      # seconds; <= 0 means spin forever
        self.declare_parameter('publish_rate', 10.0)  # Hz; keep > thrust_mixer's cmd_timeout

        self.angular_speed = self.get_parameter('angular_speed').value
        self.duration = self.get_parameter('duration').value
        publish_rate = self.get_parameter('publish_rate').value

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.start_time = self.get_clock().now()
        self._done = False
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_spin)

        self.get_logger().info(
            f'spin_node: spinning at {self.angular_speed} rad/s' +
            ('' if self.duration <= 0 else f' for {self.duration}s'))

    def publish_spin(self) -> None:
        """Timer callback: publish the spin command, or stop once `duration` elapses."""
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        if 0 < self.duration <= elapsed:
            self.cmd_pub.publish(Twist())  # zero Twist -- thrust_mixer reads this as stop
            self.get_logger().info('spin_node: duration elapsed, stopping.')
            self.timer.cancel()
            self._done = True
            return

        msg = Twist()
        msg.angular.z = self.angular_speed
        self.cmd_pub.publish(msg)


def main(args=None) -> None:
    """Initialize the ROS2 node and spin until `duration` elapses or interrupted."""
    rclpy.init(args=args)
    node = SpinNode()
    try:
        while rclpy.ok() and not node._done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info('spin_node interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
