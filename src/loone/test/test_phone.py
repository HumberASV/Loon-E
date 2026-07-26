from unittest.mock import MagicMock, patch

import pytest
from sensor_msgs.msg import NavSatFix


# ── Shared fixture ─────────────────────────────────────────────────────────────
#
# See test_thrust_mixer.py / test_busio_node.py for the same overall pattern:
# patch the names as imported into the target module, construct the real node
# under the session-scoped rclpy context from conftest.py, then swap in mocks
# for anything we want to assert on.
#
@pytest.fixture
def phone_node():
    # Replace subprocess so get_adb_devices()/route_adb() (both called from
    # __init__) never shell out to a real `adb` binary.
    subprocess_patcher = patch('loone.phone.subprocess')

    # Replace threading so the real daemon thread in __init__ never starts --
    # get_odometry() opens a real blocking socket server otherwise, which would
    # hang around across tests. get_odometry() is tested directly instead.
    threading_patcher = patch('loone.phone.threading')

    # Replace rclpy so rclpy.ok() (used inside get_odometry()'s loops) is
    # controllable per-test rather than talking to a real context.
    rclpy_patcher = patch('loone.phone.rclpy')

    subprocess_mock = subprocess_patcher.start()
    threading_patcher.start()
    rclpy_mock = rclpy_patcher.start()

    # adb devices' output: one header line + one attached device.
    subprocess_mock.check_output.return_value = b"List of devices attached\nABC123\tdevice\n"
    rclpy_mock.ok.return_value = True

    from loone.phone import Phone

    node = Phone()

    # Patch the publishers so we can inspect what gets published in tests.
    node.phone_pub = MagicMock()
    node.navsat_pub = MagicMock()

    # Patch get_logger() so log calls inside methods don't crash.
    node.get_logger = MagicMock(return_value=MagicMock())

    yield node

    rclpy_patcher.stop()
    threading_patcher.stop()
    subprocess_patcher.stop()


# ── Initial state ──────────────────────────────────────────────────────────────

class TestInitialState:
    """Phone starts with sentinel values until the first valid odometry line arrives."""

    def test_initial_values_are_sentinel(self, phone_node):
        assert phone_node.heading == -999
        assert phone_node.speed == -999
        assert phone_node.latitude == -999
        assert phone_node.longitude == -999


# ── ADB setup ──────────────────────────────────────────────────────────────────

class TestAdbSetup:
    """get_adb_devices() / route_adb() shell out to the real `adb` binary via subprocess."""

    def test_get_adb_devices_parses_serials(self, phone_node):
        # Header line is skipped; only the serial (first whitespace-separated
        # token) of each device line is kept.
        assert phone_node.get_adb_devices() == ["ABC123"]

    def test_route_adb_called_during_init(self, phone_node):
        # route_adb() runs once as part of __init__ (before this test body even
        # starts), so we can assert on it directly via the module-level mock.
        import loone.phone as phone_module

        assert phone_module.subprocess.run.call_count == 2
        phone_module.subprocess.run.assert_any_call(["adb", "reverse", "--remove-all"])
        phone_module.subprocess.run.assert_any_call(
            ["adb", "reverse", f"tcp:{phone_node.PHONE_PORT}", f"tcp:{phone_node.PORT}"])


# ── Publishers ─────────────────────────────────────────────────────────────────

class TestPublishers:
    """publish_phone() / publish_navsatfix() build a message from instance state and publish it."""

    def test_publish_phone_builds_and_publishes_data_array(self, phone_node):
        phone_node.latitude = 43.6
        phone_node.longitude = -79.4
        phone_node.speed = 2.5
        phone_node.heading = 135.0

        phone_node.publish_phone()

        phone_node.phone_pub.publish.assert_called_once()
        msg = phone_node.phone_pub.publish.call_args[0][0]
        # Float32MultiArray.data is a float32 array.array, not a list -- convert
        # before comparing, and use approx since float32 loses precision that
        # the float64 literals below have.
        assert list(msg.data) == pytest.approx([43.6, -79.4, 2.5, 135.0])

    def test_publish_navsatfix_builds_and_publishes_fix(self, phone_node):
        phone_node.latitude = 43.6
        phone_node.longitude = -79.4

        phone_node.publish_navsatfix()

        phone_node.navsat_pub.publish.assert_called_once()
        msg = phone_node.navsat_pub.publish.call_args[0][0]
        assert msg.latitude == 43.6
        assert msg.longitude == -79.4
        assert msg.altitude == 0.0
        assert msg.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN
        # Regression: navsat_transform_node needs a real frame_id to look up the
        # antenna offset (gps_link, a fixed child of base_link) -- an empty one
        # makes it log an error on every fix and skip the offset correction.
        assert msg.header.frame_id == 'gps_link'


# ── get_odometry() parsing ──────────────────────────────────────────────────────

class TestGetOdometry:
    """get_odometry() runs the blocking socket server loop; called directly here
    (the fixture prevents the real background thread from starting it) with a
    fake socket/connection and a bounded rclpy.ok() so the loops terminate."""

    @staticmethod
    def _make_conn(recv_side_effect):
        conn = MagicMock()
        conn.__enter__.return_value = conn
        conn.__exit__.return_value = False
        conn.recv.side_effect = recv_side_effect
        return conn

    def _run_with_line(self, phone_node, line_bytes):
        conn = self._make_conn([line_bytes, b""])
        server = MagicMock()
        server.accept.return_value = (conn, ('127.0.0.1', 5000))

        phone_node.publish_phone = MagicMock()
        phone_node.publish_navsatfix = MagicMock()

        with patch('loone.phone.socket') as socket_mock, \
             patch('loone.phone.rclpy') as rclpy_mock:
            socket_mock.socket.return_value = server
            # outer-accept, inner-recv(line), inner-recv(b"" -> break), outer-loop-exit
            rclpy_mock.ok.side_effect = [True, True, True, False]

            phone_node.get_odometry()

    def test_valid_line_updates_state_and_publishes(self, phone_node):
        self._run_with_line(phone_node, b"1.0,2.5,43.6,-79.4\n")

        assert phone_node.heading == 1.0
        assert phone_node.speed == 2.5
        assert phone_node.latitude == 43.6
        assert phone_node.longitude == -79.4
        phone_node.publish_phone.assert_called_once()
        phone_node.publish_navsatfix.assert_called_once()

    def test_malformed_field_count_is_skipped(self, phone_node):
        # Only 3 comma-separated fields instead of the required 4.
        self._run_with_line(phone_node, b"1.0,2.5,43.6\n")

        assert phone_node.heading == -999  # untouched sentinel
        phone_node.publish_phone.assert_not_called()
        phone_node.publish_navsatfix.assert_not_called()

    def test_non_numeric_fields_are_skipped(self, phone_node):
        # float() raises inside the try/except -> line is dropped, no publish.
        self._run_with_line(phone_node, b"a,b,c,d\n")

        assert phone_node.heading == -999  # untouched sentinel
        phone_node.publish_phone.assert_not_called()
        phone_node.publish_navsatfix.assert_not_called()
