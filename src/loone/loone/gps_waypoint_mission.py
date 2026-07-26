"""GPS-waypoint mission: a list of lat/lon points -> a single nav2 FollowWaypoints goal.

    gps_waypoints_json / gps_waypoints_file (JSON: [{"lat": .., "lon": ..}, ...])
        --> [THIS NODE] converts each point via navsat_transform_node's /fromLL service
        --> geometry_msgs/PoseStamped list, oriented toward the next waypoint
        --> nav2_msgs/action/FollowWaypoints goal (Nav2's stock waypoint_follower)

This is the MVP mission runner for Task 1 (Maneuvering and Path-Finding): it drives the
boat through a fixed list of GPS points using stock nav2_msgs + robot_localization
types, with none of the buoy/cardinal-mark/Otter awareness described in TODO.md's
"Later" section -- that intelligence belongs in a custom planner further down the line,
not in this node.
"""

import json
import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from robot_localization.srv import FromLL


class GpsWaypointMission(Node):
    """One-shot mission runner: GPS point list -> a single FollowWaypoints goal."""

    # Frame that navsat_transform_node's /fromLL points come back in. It matches
    # whatever frame_id the `odometry/filtered` topic reports -- config/navsat_transform.yaml
    # remaps that to the ZED's own odom topic. TODO(team): confirm this is really "odom"
    # once the ZED wrapper is running (ros2 topic echo <camera_name>/<zed_node_name>/odom
    # --field header.frame_id).
    WAYPOINT_FRAME_ID = 'odom'

    def __init__(self) -> None:
        super().__init__('gps_waypoint_mission')

        self.declare_parameter('gps_waypoints_json', '')
        self.declare_parameter('gps_waypoints_file', '')

        waypoints = self._load_waypoints()
        if not waypoints:
            self.get_logger().error(
                'No GPS waypoints provided (set gps_waypoints_json or gps_waypoints_file). '
                'Node will stay up but will not send a mission.')
            return

        from_ll_client = self.create_client(FromLL, 'fromLL')
        follow_waypoints_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

        self.get_logger().info('Waiting for /fromLL service (navsat_transform_node)...')
        from_ll_client.wait_for_service()

        points = []
        for lat, lon in waypoints:
            request = FromLL.Request()
            request.ll_point = GeoPoint(latitude=lat, longitude=lon)
            future = from_ll_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is None:
                self.get_logger().error(f'/fromLL call failed for ({lat}, {lon}); aborting mission.')
                return
            points.append(future.result().map_point)

        poses = self._build_poses(points)

        self.get_logger().info(
            f'Converted {len(poses)} GPS waypoints; waiting for follow_waypoints action server...')
        follow_waypoints_client.wait_for_server()

        goal = FollowWaypoints.Goal(poses=poses)
        send_goal_future = follow_waypoints_client.send_goal_async(
            goal, feedback_callback=self._on_feedback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('follow_waypoints goal rejected by Nav2.')
            return

        self.get_logger().info('follow_waypoints goal accepted; mission underway.')
        self._num_waypoints = len(poses)
        goal_handle.get_result_async().add_done_callback(self._on_result)

    def _load_waypoints(self) -> list:
        """Return [(lat, lon), ...] from gps_waypoints_json, else gps_waypoints_file."""
        raw = self.get_parameter('gps_waypoints_json').value
        if not raw:
            path = self.get_parameter('gps_waypoints_file').value
            if not path:
                return []
            try:
                with open(path, 'r') as f:
                    raw = f.read()
            except OSError as exc:
                self.get_logger().error(f'Could not read gps_waypoints_file "{path}": {exc}')
                return []

        try:
            points = json.loads(raw)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'gps_waypoints JSON is invalid: {exc}')
            return []

        return [(float(p['lat']), float(p['lon'])) for p in points]

    def _build_poses(self, points: list) -> list:
        """Turn /fromLL Points into PoseStamped, each oriented toward the next point."""
        poses = []
        n = len(points)
        prev_yaw = 0.0
        for i, point in enumerate(points):
            if i < n - 1:
                nxt = points[i + 1]
                yaw = math.atan2(nxt.y - point.y, nxt.x - point.x)
            else:
                yaw = prev_yaw  # last waypoint: hold the previous segment's heading
            prev_yaw = yaw

            pose = PoseStamped()
            pose.header.frame_id = self.WAYPOINT_FRAME_ID
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position = point
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            poses.append(pose)
        return poses

    def _on_feedback(self, feedback_msg) -> None:
        current = feedback_msg.feedback.current_waypoint
        self.get_logger().info(f'Heading to waypoint {current + 1}/{self._num_waypoints}')

    def _on_result(self, future) -> None:
        result = future.result().result
        if result.missed_waypoints:
            self.get_logger().warn(f'Mission finished; missed waypoints: {list(result.missed_waypoints)}')
        else:
            self.get_logger().info('Mission complete -- all GPS waypoints reached.')


def main(args=None) -> None:
    """Initialize the ROS2 node and spin."""
    rclpy.init(args=args)
    node = GpsWaypointMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('gps_waypoint_mission interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
