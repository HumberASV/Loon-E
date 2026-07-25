"""Thrust mixer: nav2 /cmd_vel -> normalized [prop_l, prop_r, rudder] commands.

This node is the "control" layer of the chained-controls stack:

    nav2 controller_server --> /cmd_vel (geometry_msgs/Twist)
        --> [THIS NODE] mixes surge + yaw into three servo fractions
        --> /asv_forward_controller/commands (std_msgs/Float64MultiArray)
        --> forward_command_controller (ros2_control) --> hardware command interfaces

Why a Python node instead of a C++ ros2_control controller?
    The team chose to keep the tunable control math in Python (easy to edit, and it
    reuses the intent of the old motor.py PID). ros2_control still owns the hardware
    command interface; this node just feeds a stock ForwardCommandController.
    If/when you want the mixing to run inside the real-time controller_manager loop,
    port this logic into a C++ ControllerInterface and delete this node -- nav2 and
    the URDF do not change.

Output convention (matches the old motor.py servo fractions):
    * propellers: 0.0 = full reverse, 0.5 = neutral/stop, 1.0 = full forward
    * rudder:     0.0 = full one way, `center` (~0.55) = straight, 1.0 = full other way
    The pca9685_driver node converts these fractions to PCA9685 pulse widths.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


def clamp(value: float, low: float, high: float) -> float:
    """Clamp `value` into [low, high]."""
    return max(low, min(high, value))


class ThrustMixer(Node):
    """Convert a body-frame velocity command into three normalized servo fractions."""

    def __init__(self) -> None:
        super().__init__('thrust_mixer')

        # ---- Parameters (edit these live with `ros2 param set` or in ros2_control launch) ----
        # Neutral / center points. `center` mirrors motor.py's rudder center (config.yaml: 0.55).
        self.declare_parameter('prop_neutral', 0.5)   # propeller fraction that produces no thrust
        self.declare_parameter('rudder_center', 0.55)  # rudder fraction that points straight ahead

        # Gains: how strongly each cmd_vel component moves the outputs.
        # TODO(team): tune on the water. Start small so the boat is not twitchy.
        #   surge_gain: fraction added per (m/s) of forward speed request.
        #   yaw_gain:   differential-thrust fraction added per (rad/s) of yaw request.
        #   rudder_gain: rudder fraction added per (rad/s) of yaw request.
        self.declare_parameter('surge_gain', 0.5)
        self.declare_parameter('yaw_gain', 0.3)
        self.declare_parameter('rudder_gain', 0.5)

        # Safety: how much a propeller may deviate from neutral (0.5 -> [0.5-limit, 0.5+limit]).
        # TODO(team): raise once you trust the boat. 0.3 keeps it gentle for first tests.
        self.declare_parameter('prop_limit', 0.3)

        # If no /cmd_vel arrives within this many seconds, fall back to neutral (dead-man switch).
        self.declare_parameter('cmd_timeout', 0.5)
        # Output rate to the controller (Hz). Keep >= nav2 controller_frequency.
        self.declare_parameter('publish_rate', 20.0)

        self.prop_neutral = self.get_parameter('prop_neutral').value
        self.rudder_center = self.get_parameter('rudder_center').value
        self.surge_gain = self.get_parameter('surge_gain').value
        self.yaw_gain = self.get_parameter('yaw_gain').value
        self.rudder_gain = self.get_parameter('rudder_gain').value
        self.prop_limit = self.get_parameter('prop_limit').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        publish_rate = self.get_parameter('publish_rate').value

        # ---- ROS wiring ----
        # NOTE: nav2's controller_server publishes geometry_msgs/Twist on /cmd_vel by default on
        # Humble. If you enable_stamped_cmd_vel / use a newer nav2, switch this to TwistStamped.
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        # The topic name MUST match the ForwardCommandController name in ros2_control.yaml:
        #   <controller_name>/commands  ==  asv_forward_controller/commands
        self.cmd_pub = self.create_publisher(
            Float64MultiArray, 'asv_forward_controller/commands', 10)

        # Latest command + when we received it (for the dead-man timeout).
        self.last_cmd = Twist()
        self.last_cmd_time = self.get_clock().now()

        # Publish on a fixed timer so the hardware always gets a fresh (or neutral) command.
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_commands)

        self.get_logger().info('thrust_mixer ready, waiting for /cmd_vel...')

    def cmd_callback(self, msg: Twist) -> None:
        """Store the newest velocity command and stamp its arrival time."""
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def mix(self, cmd: Twist) -> list:
        """Map a Twist to [prop_l, prop_r, rudder] fractions.

        This is the piece most worth tuning/replacing. The default is a simple
        differential-thrust + rudder mix -- the same idea as motor.py's drive():
        forward speed sets a base throttle, yaw rate biases the two propellers
        apart and deflects the rudder.

        Args:
            cmd: desired body-frame velocity. Uses linear.x (surge, m/s) and
                 angular.z (yaw rate, rad/s); sway (linear.y) is ignored -- the
                 boat is under-actuated.

        Returns:
            [prop_l, prop_r, rudder] as fractions (see module docstring).
        """
        surge = cmd.linear.x
        yaw = cmd.angular.z

        base = self.prop_neutral + self.surge_gain * surge   # common forward/reverse throttle
        diff = self.yaw_gain * yaw                            # differential thrust for turning

        prop_lo = self.prop_neutral - self.prop_limit
        prop_hi = self.prop_neutral + self.prop_limit
        prop_l = clamp(base + diff, prop_lo, prop_hi)
        prop_r = clamp(base - diff, prop_lo, prop_hi)

        rudder = clamp(self.rudder_center + self.rudder_gain * yaw, 0.0, 1.0)

        # TODO(team): things worth adding once the basics work --
        #   * yaw deadband so tiny commands don't chatter the servos
        #   * slew-rate limiting (don't jump the throttle in one tick)
        #   * reverse handling (props below 0.5 -- confirm your ESCs are bidirectional)
        #   * saturation priority (favor turning vs. speed when both saturate)
        #   * OPTIONAL inner heading PID for current/wind rejection: subscribe /odom,
        #     compare measured yaw to an integrated heading setpoint, and add the PID
        #     output into `diff`/`rudder`. Port kp/ki/kd + clamp from motor.py drive().
        #     nav2 already closes the heading loop at the trajectory level, so this is
        #     off by default; add it only if drift is a problem on the water.
        return [prop_l, prop_r, rudder]

    def publish_commands(self) -> None:
        """Timer callback: publish the mixed command, or neutral if the command is stale."""
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if age > self.cmd_timeout:
            # Dead-man: no fresh command -> stop the props and center the rudder.
            fractions = [self.prop_neutral, self.prop_neutral, self.rudder_center]
        else:
            fractions = self.mix(self.last_cmd)

        msg = Float64MultiArray()
        # ORDER MATTERS: must match the `joints:` order in ros2_control.yaml
        # (asv_forward_controller) -> [prop_l_joint, prop_r_joint, rudder_joint].
        msg.data = [float(x) for x in fractions]
        self.cmd_pub.publish(msg)


def main(args=None) -> None:
    """Initialize the ROS2 node and spin."""
    rclpy.init(args=args)
    node = ThrustMixer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('thrust_mixer interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
