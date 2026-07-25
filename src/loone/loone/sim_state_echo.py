"""Open-loop joint state echo for simulation runs.

topic_based_ros2_control needs BOTH halves of its topic pair:

    * it PUBLISHES command-interface values on /asv/joint_commands
    * it SUBSCRIBES /asv/joint_states to fill its state interfaces

On the real boat, pca9685_driver.py provides that second half -- it echoes the
commands it just wrote to the servos (see its `publish_state`). In simulation
pca9685_driver is not running (no I2C bus), so nothing publishes /asv/joint_states,
the state interfaces stay NaN, joint_state_broadcaster publishes NaN, and
robot_state_publisher floods the log with TF errors.

This node fills that gap and NOTHING else: commands in, same values back out as
state. It is deliberately the same open-loop echo the real driver does, so the
sim and hardware topic graphs are identical -- the only thing that changes
between them is which node subscribes /asv/joint_commands
(pca9685_driver.py on the boat, ros2_bridge.py inside Isaac Sim).

If you later want true feedback in sim (real rudder angle from the physics
articulation rather than the commanded value), publish /asv/joint_states from
Isaac Sim instead of running this node -- but note the sim's USD joint names must
be remapped to the URDF names first, or topic_based_ros2_control will ignore them.

Usage:
    ros2 run loone sim_state_echo
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

# Joint names as declared in loone_asv.urdf.xacro. Publishing state for a joint
# ros2_control does not know about is harmless; MISSING one leaves that state
# interface at NaN, so always echo all three.
JOINTS = ['prop_l_joint', 'prop_r_joint', 'rudder_joint']

# Neutral fractions, matching thrust_mixer.py / pca9685_driver.py defaults. These
# seed the state before the first command arrives so ros2_control never sees NaN.
NEUTRAL = {
    'prop_l_joint': 0.5,
    'prop_r_joint': 0.5,
    'rudder_joint': 0.55,
}


class SimStateEcho(Node):
    """Mirror /asv/joint_commands back onto /asv/joint_states."""

    def __init__(self) -> None:
        super().__init__('sim_state_echo')

        # Topic names must match the hardware params in loone_asv.urdf.xacro.
        self.declare_parameter('joint_commands_topic', '/asv/joint_commands')
        self.declare_parameter('joint_states_topic', '/asv/joint_states')
        # Keep publishing even when no command arrives, so ros2_control's state
        # interfaces stay fresh and the controller_manager does not stall.
        self.declare_parameter('publish_rate', 50.0)

        commands_topic = self.get_parameter('joint_commands_topic').value
        states_topic = self.get_parameter('joint_states_topic').value
        publish_rate = self.get_parameter('publish_rate').value

        # Last commanded fraction per joint; starts neutral.
        self.positions = dict(NEUTRAL)

        self.sub = self.create_subscription(
            JointState, commands_topic, self.command_callback, 10)
        self.pub = self.create_publisher(JointState, states_topic, 10)
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_state)

        self.get_logger().info(
            f'sim_state_echo ready: {commands_topic} -> {states_topic}')

    def command_callback(self, msg: JointState) -> None:
        """Store the commanded positions, matched by joint name."""
        # Match by name rather than index: the publisher is free to reorder.
        for name, position in zip(msg.name, msg.position):
            if name in self.positions:
                self.positions[name] = position

    def publish_state(self) -> None:
        """Publish the stored commands back as the measured state."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINTS)
        msg.position = [float(self.positions[name]) for name in JOINTS]
        self.pub.publish(msg)


def main(args=None) -> None:
    """Initialize the ROS2 node and spin."""
    rclpy.init(args=args)
    node = SimStateEcho()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('sim_state_echo interrupted by user.')
    except ExternalShutdownException:
        # SIGTERM -- how `ros2 launch` stops its nodes. rclpy has already torn the
        # context down by the time this is raised, so calling shutdown() again below
        # would raise RCLError and make a clean stop look like a crash.
        node.get_logger().info('sim_state_echo shut down externally.')
    except Exception:
        # On Humble, SIGTERM often invalidates the context while spin() is building
        # its wait set, which surfaces as a bare RCLError instead of the exception
        # above (and rclpy exposes no public RCLError to catch by type). If the
        # context is already down, this is that shutdown race -- exit quietly.
        # Otherwise it is a genuine fault and must not be swallowed.
        if rclpy.ok():
            raise
        node.get_logger().info('sim_state_echo shut down externally.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
