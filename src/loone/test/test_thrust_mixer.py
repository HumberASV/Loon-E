from unittest.mock import MagicMock

import pytest
from geometry_msgs.msg import Twist

from loone.thrust_mixer import clamp


# ── Shared fixture ─────────────────────────────────────────────────────────────
#
# thrust_mixer.py has no hardware imports and __init__ never blocks, so there is
# nothing to patch at module scope -- just build the node under the real
# session-scoped ROS context from conftest.py and swap in mocks for the
# publisher/logger we want to inspect.
#
@pytest.fixture
def mixer_node():
    from loone.thrust_mixer import ThrustMixer

    node = ThrustMixer()
    node.cmd_pub = MagicMock()
    node.get_logger = MagicMock(return_value=MagicMock())

    yield node


def _mock_clock_delta(node, delta_seconds):
    """Make node.get_clock().now() - node.last_cmd_time report a fixed age,
    without depending on real wall-clock timing."""
    fake_delta = MagicMock()
    fake_delta.nanoseconds = int(delta_seconds * 1e9)
    fake_now = MagicMock()
    fake_now.__sub__.return_value = fake_delta
    node.get_clock = MagicMock(return_value=MagicMock(now=MagicMock(return_value=fake_now)))


# ── Pure math tests ────────────────────────────────────────────────────────────

class TestClamp:
    """clamp() restricts a value to [low, high]."""

    def test_within_range_unchanged(self):
        assert clamp(0.5, 0.0, 1.0) == 0.5

    def test_below_low_clamps(self):
        assert clamp(-1.0, 0.0, 1.0) == 0.0

    def test_above_high_clamps(self):
        assert clamp(2.0, 0.0, 1.0) == 1.0


# ── mix() tests ─────────────────────────────────────────────────────────────────

class TestMix:
    """mix() maps a Twist to [prop_l, prop_r, rudder] fractions using the
    node's (default) surge/yaw/rudder gains and safety limits."""

    def test_zero_cmd_returns_neutral_triple(self, mixer_node):
        result = mixer_node.mix(Twist())
        assert result == [
            mixer_node.prop_neutral, mixer_node.prop_neutral, mixer_node.rudder_center,
        ]

    def test_positive_surge_raises_both_props_equally(self, mixer_node):
        cmd = Twist()
        cmd.linear.x = 0.2  # no yaw -> symmetric throttle bump, rudder stays centered
        prop_l, prop_r, rudder = mixer_node.mix(cmd)
        assert prop_l == prop_r > mixer_node.prop_neutral
        assert rudder == mixer_node.rudder_center

    def test_positive_yaw_biases_left_prop_higher_and_rudder_off_center(self, mixer_node):
        cmd = Twist()
        cmd.angular.z = 0.5
        prop_l, prop_r, rudder = mixer_node.mix(cmd)
        assert prop_l > prop_r
        assert rudder > mixer_node.rudder_center

    def test_negative_yaw_biases_right_prop_higher_and_rudder_other_way(self, mixer_node):
        cmd = Twist()
        cmd.angular.z = -0.5
        prop_l, prop_r, rudder = mixer_node.mix(cmd)
        assert prop_r > prop_l
        assert rudder < mixer_node.rudder_center

    def test_large_surge_saturates_props_at_prop_limit(self, mixer_node):
        cmd = Twist()
        cmd.linear.x = 100.0
        prop_l, prop_r, _ = mixer_node.mix(cmd)
        assert prop_l == mixer_node.prop_neutral + mixer_node.prop_limit
        assert prop_r == mixer_node.prop_neutral + mixer_node.prop_limit

    def test_large_negative_yaw_clamps_rudder_to_zero(self, mixer_node):
        cmd = Twist()
        cmd.angular.z = -100.0
        _, _, rudder = mixer_node.mix(cmd)
        assert rudder == 0.0

    def test_large_positive_yaw_clamps_rudder_to_one(self, mixer_node):
        cmd = Twist()
        cmd.angular.z = 100.0
        _, _, rudder = mixer_node.mix(cmd)
        assert rudder == 1.0


# ── Callback tests ──────────────────────────────────────────────────────────────

class TestCmdCallback:

    def test_stores_latest_command_and_time(self, mixer_node):
        sentinel_time = MagicMock()
        mixer_node.get_clock = MagicMock(
            return_value=MagicMock(now=MagicMock(return_value=sentinel_time)))

        msg = Twist()
        msg.linear.x = 0.3
        mixer_node.cmd_callback(msg)

        assert mixer_node.last_cmd is msg
        assert mixer_node.last_cmd_time is sentinel_time


# ── Timer / dead-man tests ───────────────────────────────────────────────────────

class TestPublishCommands:
    """publish_commands() is the timer callback: mix a fresh command, or go neutral if stale."""

    def test_publishes_mixed_command_when_fresh(self, mixer_node):
        _mock_clock_delta(mixer_node, delta_seconds=0.1)  # < default cmd_timeout (0.5s)
        mixer_node.last_cmd = Twist()
        mixer_node.last_cmd.linear.x = 0.2
        mixer_node.mix = MagicMock(return_value=[0.6, 0.6, 0.55])

        mixer_node.publish_commands()

        mixer_node.mix.assert_called_once_with(mixer_node.last_cmd)
        mixer_node.cmd_pub.publish.assert_called_once()
        msg = mixer_node.cmd_pub.publish.call_args[0][0]
        # Float64MultiArray.data is an array.array, not a list.
        assert list(msg.data) == [0.6, 0.6, 0.55]

    def test_publishes_neutral_and_skips_mix_when_stale(self, mixer_node):
        _mock_clock_delta(mixer_node, delta_seconds=10.0)  # > default cmd_timeout (0.5s)
        mixer_node.mix = MagicMock()

        mixer_node.publish_commands()

        mixer_node.mix.assert_not_called()
        msg = mixer_node.cmd_pub.publish.call_args[0][0]
        # Float64MultiArray.data is an array.array, not a list.
        assert list(msg.data) == [
            mixer_node.prop_neutral, mixer_node.prop_neutral, mixer_node.rudder_center,
        ]
