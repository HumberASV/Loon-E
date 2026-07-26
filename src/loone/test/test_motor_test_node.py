from unittest.mock import MagicMock

import pytest


# ── Shared fixture ─────────────────────────────────────────────────────────────
#
# Same approach as test_spin_node.py: MotorTestNode.__init__ never blocks, so it's
# built under the real session-scoped ROS context from conftest.py, with the timer
# swapped for a mock so publish_toggle()'s timer.cancel() call doesn't need a real
# timer object.
#
@pytest.fixture
def motor_test_node():
    from loone.motor_test_node import MotorTestNode

    node = MotorTestNode()
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


class TestPublishToggle:
    """publish_toggle() is the timer callback: alternate thrust/neutral, then stop at cycles."""

    def test_publishes_thrust_during_on_phase(self, motor_test_node):
        motor_test_node.on_duration = 2.0
        motor_test_node.off_duration = 2.0
        motor_test_node.cycles = 3
        _mock_elapsed(motor_test_node, seconds=1.0)  # within the first on-phase

        motor_test_node.publish_toggle()

        msg = motor_test_node.cmd_pub.publish.call_args[0][0]
        assert msg.linear.x == motor_test_node.thrust
        motor_test_node.timer.cancel.assert_not_called()
        assert motor_test_node._done is False

    def test_publishes_neutral_during_off_phase(self, motor_test_node):
        motor_test_node.on_duration = 2.0
        motor_test_node.off_duration = 2.0
        motor_test_node.cycles = 3
        _mock_elapsed(motor_test_node, seconds=3.0)  # within the first off-phase (2-4s)

        motor_test_node.publish_toggle()

        msg = motor_test_node.cmd_pub.publish.call_args[0][0]
        assert msg.linear.x == 0.0
        motor_test_node.timer.cancel.assert_not_called()
        assert motor_test_node._done is False

    def test_cycles_through_multiple_on_off_periods(self, motor_test_node):
        motor_test_node.on_duration = 2.0
        motor_test_node.off_duration = 2.0
        motor_test_node.cycles = 3
        _mock_elapsed(motor_test_node, seconds=9.0)  # 2 full periods (8s) + 1s into 3rd on-phase

        motor_test_node.publish_toggle()

        msg = motor_test_node.cmd_pub.publish.call_args[0][0]
        assert msg.linear.x == motor_test_node.thrust
        assert motor_test_node._done is False

    def test_stops_once_all_cycles_complete(self, motor_test_node):
        motor_test_node.on_duration = 2.0
        motor_test_node.off_duration = 2.0
        motor_test_node.cycles = 3
        _mock_elapsed(motor_test_node, seconds=12.0)  # 3 periods (12s) fully elapsed

        motor_test_node.publish_toggle()

        msg = motor_test_node.cmd_pub.publish.call_args[0][0]
        assert msg.linear.x == 0.0
        motor_test_node.timer.cancel.assert_called_once()
        assert motor_test_node._done is True

    def test_non_positive_cycles_means_repeat_forever(self, motor_test_node):
        motor_test_node.on_duration = 2.0
        motor_test_node.off_duration = 2.0
        motor_test_node.cycles = 0
        _mock_elapsed(motor_test_node, seconds=1e6)

        motor_test_node.publish_toggle()

        motor_test_node.timer.cancel.assert_not_called()
        assert motor_test_node._done is False
