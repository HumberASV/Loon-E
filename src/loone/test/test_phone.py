# Tests for phone.py

from unittest.mock import MagicMock, patch, call  # noqa: F401
import numpy as np
import pytest

# Add a fixture to create mocks to create a 
# node without actually running it. 
# This is to test the phone node without actually running it.
@pytest.fixture
def phone_node():
    # patch rclpy to a mock object
    # this is to make a object that can be used to test 
    # the phone node without actually running it
    rclpy_patcher = patch('loone.phone.rclpy')
    socket_patcher = patch('loone.phone.socket')
    subprocess_patcher = patch('loone.phone.subprocess')

    # start the patcher and get the mock object
    rclpy_mock = rclpy_patcher.start()
    socket_mock = socket_patcher.start()
    subprocess_mock = subprocess_patcher.start()

    # Phone.__init__ calls super().__init__('Phone_Sub') which is rclpy.Node.__init__ 
    # we need to override the __init__ method of rclpy.Node to do nothing so that 
    # we can create a Phone object without actually creating a node
    rclpy_mock.node.Node.__init__ = MagicMock(return_value=None)

    from loone.phone import Phone

    node = Phone()

    ## patch the subscriber to a mock object
    node.subscription_ = MagicMock()

    # patch logger to a mock object
    node.get_logger = MagicMock(return_value=MagicMock())

    # Attach the active mocks to the node so individual tests can configure them.
    # Tests that test get_odometry() need to control rclpy.ok() and socket.socket().
    node._rclpy_mock = rclpy_mock
    node._socket_mock = socket_mock

    yield node # test runs here

    # stop the patcher
    rclpy_patcher.stop()
    socket_patcher.stop()
    subprocess_patcher.stop()

class TestPhone:
    def test_publish(self, phone_node):
        """
        Test the publish method of the Phone node.
        """
        # call the publish method
        phone_node.latitude = 1.0
        phone_node.longitude = 2.0
        phone_node.speed = 3.0
        phone_node.heading = 4.0

        phone_node.publish()

        # check that the publisher was called with the correct message
        expected_msg = [phone_node.latitude, phone_node.longitude, phone_node.speed, phone_node.heading]
        phone_node.publisher_.publish.assert_called_once()
        published_msg = phone_node.publisher_.publish.call_args[0][0]
        assert published_msg.data == expected_msg
    
    def test_get_odometry(self, phone_node):
        """
        Test the get_odometry method of the Phone node.
        """
        # get_odometry() calls socket.socket(...) to create a local `server` variable —
        # it never uses self.socket. So we configure the module-level socket mock
        # (exposed from the fixture) to control what socket.socket(...) returns.
        server_mock = MagicMock()
        phone_node._socket_mock.socket.return_value = server_mock

        conn_mock = MagicMock()
        server_mock.accept.return_value = (conn_mock, MagicMock())

        # recv is called twice: once with real data, then with b"" to break the inner loop.
        # side_effect lets you give different return values on successive calls.
        conn_mock.recv.side_effect = [b"1.0,2.0,3.0,4.0\n", b""]

        # rclpy.ok() drives both while loops. We give it exactly enough Trues to
        # enter and process one message, then False to exit the outer loop.
        # Call order: outer-enter, inner-enter, inner-continue, outer-exit
        phone_node._rclpy_mock.ok.side_effect = [True, True, True, False]

        phone_node.get_odometry()

        assert phone_node.heading == 1.0
        assert phone_node.speed == 2.0
        assert phone_node.latitude == 3.0
        assert phone_node.longitude == 4.0

    def test_get_odometry_invalid_data(self, phone_node):
        """
        Test the get_odometry method with invalid data.
        """
        server_mock = MagicMock()
        phone_node._socket_mock.socket.return_value = server_mock

        conn_mock = MagicMock()
        server_mock.accept.return_value = (conn_mock, MagicMock())

        # "invalid,data" has 2 parts — len(parts) != 4 so fields stay as nan.
        # Second recv returns b"" to break the inner loop.
        conn_mock.recv.side_effect = [b"invalid,data\n", b""]
        phone_node._rclpy_mock.ok.side_effect = [True, True, True, False]

        phone_node.get_odometry()

        assert np.isnan(phone_node.heading)
        assert np.isnan(phone_node.speed)
        assert np.isnan(phone_node.latitude)
        assert np.isnan(phone_node.longitude)

    def test_get_odometry_partial_data(self, phone_node):
        """
        Test the get_odometry method with partial data.
        """
        server_mock = MagicMock()
        phone_node._socket_mock.socket.return_value = server_mock

        conn_mock = MagicMock()
        server_mock.accept.return_value = (conn_mock, MagicMock())

        # "1.0,2.0" has only 2 parts — skipped, fields stay as nan.
        conn_mock.recv.side_effect = [b"1.0,2.0\n", b""]
        phone_node._rclpy_mock.ok.side_effect = [True, True, True, False]

        phone_node.get_odometry()

        assert np.isnan(phone_node.heading)
        assert np.isnan(phone_node.speed)
        assert np.isnan(phone_node.latitude)
        assert np.isnan(phone_node.longitude)

    def test_get_odometry_empty_data(self, phone_node):
        """
        Test the get_odometry method with empty data.
        """
        server_mock = MagicMock()
        phone_node._socket_mock.socket.return_value = server_mock

        conn_mock = MagicMock()
        server_mock.accept.return_value = (conn_mock, MagicMock())

        # b"" triggers `if not data: break` immediately — inner loop exits right away.
        # Only three rclpy.ok() calls: outer-enter, inner-enter, outer-exit.
        conn_mock.recv.return_value = b""
        phone_node._rclpy_mock.ok.side_effect = [True, True, False]

        phone_node.get_odometry()

        assert np.isnan(phone_node.heading)
        assert np.isnan(phone_node.speed)
        assert np.isnan(phone_node.latitude)
        assert np.isnan(phone_node.longitude)

