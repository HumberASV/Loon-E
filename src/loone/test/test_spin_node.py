from unittest.mock import MagicMock

import pytest


# ── Shared fixture ─────────────────────────────────────────────────────────────
#
# SpinNode.__init__ never blocks (just sets up a publisher/timer), so it's built
# under the real session-scoped ROS context from conftest.py, same as thrust_mixer's
# tests -- with the timer swapped for a mock so publish_spin()'s timer.cancel() call
# doesn't need a real timer object.
#
@pytest.fixture
def spin_node():
    from loone.spin_node import SpinNode

    node = SpinNode()
    node.cmd_pub = MagicMock()
    node.get_logger = MagicMock(return_value=MagicMock())
    node.timer = MagicMock()
    yield node


def _mock_elapsed(node, seconds):
    """Make (node.get_clock().now() - node.start_time) report a fixed elapsed duration."""
    fake_delta = MagicMock()
    fake_delta.nanoseconds = int(seconds * 1e9)
    fake_now = MagicMock()
    fake_now.__sub__.return_value = fake_delta
    node.get_clock = MagicMock(return_value=MagicMock(now=MagicMock(return_value=fake_now)))


class TestPublishSpin:
    """publish_spin() is the timer callback: spin at a constant yaw rate, or stop at duration."""

    def test_publishes_angular_speed_before_duration_elapses(self, spin_node):
        spin_node.duration = 20.0
        _mock_elapsed(spin_node, seconds=5.0)

        spin_node.publish_spin()

        spin_node.cmd_pub.publish.assert_called_once()
        msg = spin_node.cmd_pub.publish.call_args[0][0]
        assert msg.angular.z == spin_node.angular_speed
        spin_node.timer.cancel.assert_not_called()
        assert spin_node._done is False

    def test_publishes_zero_and_stops_once_duration_elapses(self, spin_node):
        spin_node.duration = 20.0
        _mock_elapsed(spin_node, seconds=25.0)

        spin_node.publish_spin()

        msg = spin_node.cmd_pub.publish.call_args[0][0]
        assert msg.angular.z == 0.0
        assert msg.linear.x == 0.0
        spin_node.timer.cancel.assert_called_once()
        assert spin_node._done is True

    def test_non_positive_duration_means_spin_forever(self, spin_node):
        spin_node.duration = 0.0
        _mock_elapsed(spin_node, seconds=1e6)

        spin_node.publish_spin()

        msg = spin_node.cmd_pub.publish.call_args[0][0]
        assert msg.angular.z == spin_node.angular_speed
        spin_node.timer.cancel.assert_not_called()
        assert spin_node._done is False
