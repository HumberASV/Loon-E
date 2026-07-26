import json
import math
from unittest.mock import MagicMock

import pytest
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import Point
from rclpy.parameter import Parameter
from robot_localization.srv import FromLL


# ── Shared fixture ─────────────────────────────────────────────────────────────
#
# GpsWaypointMission.__init__ blocks on /fromLL and follow_waypoints if it has
# waypoints to send -- but with both gps_waypoints_json and gps_waypoints_file
# left at their declared '' defaults, _load_waypoints() returns [] and __init__
# logs an error and returns *before* touching any service/action client. That
# early-return is what makes it safe to construct for real here (real session-
# scoped ROS context from conftest.py), instead of needing to mock rclpy itself.
#
@pytest.fixture
def mission_node():
    from loone.gps_waypoint_mission import GpsWaypointMission

    node = GpsWaypointMission()
    node.get_logger = MagicMock(return_value=MagicMock())
    yield node


def _set_param(node, name, value):
    node.set_parameters([Parameter(name, value=value)])


# ── _load_waypoints() tests ──────────────────────────────────────────────────────

class TestLoadWaypoints:
    """_load_waypoints() turns gps_waypoints_json/_file into [(lat, lon), ...]."""

    def test_returns_empty_when_neither_param_set(self, mission_node):
        assert mission_node._load_waypoints() == []

    def test_parses_gps_waypoints_json_param(self, mission_node):
        _set_param(mission_node, 'gps_waypoints_json',
                   json.dumps([{'lat': 43.65, 'lon': -79.38}, {'lat': 43.66, 'lon': -79.39}]))
        assert mission_node._load_waypoints() == [(43.65, -79.38), (43.66, -79.39)]

    def test_json_param_takes_precedence_over_file_param(self, mission_node, tmp_path):
        waypoints_file = tmp_path / 'course.json'
        waypoints_file.write_text(json.dumps([{'lat': 1.0, 'lon': 2.0}]))
        _set_param(mission_node, 'gps_waypoints_file', str(waypoints_file))
        _set_param(mission_node, 'gps_waypoints_json', json.dumps([{'lat': 9.0, 'lon': 9.0}]))

        assert mission_node._load_waypoints() == [(9.0, 9.0)]

    def test_falls_back_to_file_when_json_param_empty(self, mission_node, tmp_path):
        waypoints_file = tmp_path / 'course.json'
        waypoints_file.write_text(json.dumps([{'lat': 43.59, 'lon': -79.51}]))
        _set_param(mission_node, 'gps_waypoints_file', str(waypoints_file))

        assert mission_node._load_waypoints() == [(43.59, -79.51)]

    def test_invalid_json_returns_empty_list(self, mission_node):
        _set_param(mission_node, 'gps_waypoints_json', 'not valid json')
        assert mission_node._load_waypoints() == []

    def test_unreadable_file_returns_empty_list(self, mission_node, tmp_path):
        _set_param(mission_node, 'gps_waypoints_file', str(tmp_path / 'does_not_exist.json'))
        assert mission_node._load_waypoints() == []


# ── /fromLL request regression test ──────────────────────────────────────────────
#
# Bench-test 2026-07-26 caught this live: the __init__ mission loop builds
# FromLL.Request().ll_point straight from a (lat, lon), and it used to construct it
# as a sensor_msgs/NavSatFix. FromLL.srv's ll_point field is actually a
# geographic_msgs/GeoPoint, so that raised an AssertionError from rclpy's generated
# setter and crashed the node before it ever reached the mission logic below --
# undetected by the tests above since request-building happens inline in __init__,
# not in a helper method they exercise.
#
def test_from_ll_request_accepts_a_geopoint_lat_lon():
    request = FromLL.Request()
    request.ll_point = GeoPoint(latitude=43.59, longitude=-79.51)
    assert request.ll_point.latitude == pytest.approx(43.59)
    assert request.ll_point.longitude == pytest.approx(-79.51)


# ── _build_poses() tests ─────────────────────────────────────────────────────────

class TestBuildPoses:
    """_build_poses() turns /fromLL Points into PoseStamped oriented toward the next point."""

    def test_single_point_defaults_to_zero_yaw(self, mission_node):
        points = [Point(x=1.0, y=2.0, z=0.0)]
        poses = mission_node._build_poses(points)

        assert len(poses) == 1
        assert poses[0].header.frame_id == mission_node.WAYPOINT_FRAME_ID
        assert poses[0].pose.position == points[0]
        assert poses[0].pose.orientation.z == pytest.approx(0.0)
        assert poses[0].pose.orientation.w == pytest.approx(1.0)

    def test_orients_each_pose_toward_the_next_point(self, mission_node):
        # Heading due "north" in the odom frame (+y): yaw = pi/2.
        points = [Point(x=0.0, y=0.0, z=0.0), Point(x=0.0, y=1.0, z=0.0)]
        poses = mission_node._build_poses(points)

        expected_yaw = math.pi / 2
        assert poses[0].pose.orientation.z == pytest.approx(math.sin(expected_yaw / 2))
        assert poses[0].pose.orientation.w == pytest.approx(math.cos(expected_yaw / 2))

    def test_last_pose_holds_final_segment_heading(self, mission_node):
        # Leg 1 (0,0)->(1,0) heads east (yaw=0); leg 2 (1,0)->(1,5) heads north (yaw=pi/2).
        # The final waypoint has no outgoing leg, so it should keep leg 2's heading.
        points = [
            Point(x=0.0, y=0.0, z=0.0),
            Point(x=1.0, y=0.0, z=0.0),
            Point(x=1.0, y=5.0, z=0.0),
        ]
        poses = mission_node._build_poses(points)

        expected_yaw = math.pi / 2
        assert poses[2].pose.orientation.z == pytest.approx(math.sin(expected_yaw / 2))
        assert poses[2].pose.orientation.w == pytest.approx(math.cos(expected_yaw / 2))

    def test_all_poses_use_the_waypoint_frame_id(self, mission_node):
        points = [Point(x=0.0, y=0.0, z=0.0), Point(x=1.0, y=1.0, z=0.0)]
        poses = mission_node._build_poses(points)

        assert all(pose.header.frame_id == mission_node.WAYPOINT_FRAME_ID for pose in poses)
