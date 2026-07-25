from unittest.mock import MagicMock

import pytest
from sensor_msgs.msg import BatteryState


# ── Shared fixture ─────────────────────────────────────────────────────────────
#
# battery_node.py has no hardware imports at all (just rclpy/Node plus the two
# message types), so unlike busio_node.py/phone.py there is nothing to patch at
# module scope -- just build the node under the real session-scoped ROS context
# from conftest.py and swap in mocks for the publishers we want to inspect.
#
@pytest.fixture
def battery_node():
    from loone.battery_node import BatteryNode

    node = BatteryNode()

    node.prop_l_pub = MagicMock()
    node.prop_r_pub = MagicMock()
    node.main_pub = MagicMock()
    node.get_logger = MagicMock(return_value=MagicMock())

    yield node


# ── _to_battery_state() tests ───────────────────────────────────────────────────

class TestToBatteryState:
    """_to_battery_state() converts a raw voltage into percentage + health."""

    def test_mid_range_voltage_reports_good_health(self, battery_node):
        msg = battery_node._to_battery_state(18.0, min_v=15.0, max_v=21.0, location='prop_l')

        assert 0.0 < msg.percentage < 1.0
        assert msg.voltage == 18.0
        assert msg.present is True
        assert msg.location == 'prop_l'
        assert msg.power_supply_health == BatteryState.POWER_SUPPLY_HEALTH_GOOD

    def test_below_min_voltage_clamps_percentage_and_reports_dead(self, battery_node):
        msg = battery_node._to_battery_state(10.0, min_v=15.0, max_v=21.0, location='prop_l')

        assert msg.percentage == 0.0
        assert msg.power_supply_health == BatteryState.POWER_SUPPLY_HEALTH_DEAD

    def test_above_max_voltage_clamps_percentage_and_reports_overvoltage(self, battery_node):
        msg = battery_node._to_battery_state(25.0, min_v=15.0, max_v=21.0, location='prop_l')

        assert msg.percentage == 1.0
        assert msg.power_supply_health == BatteryState.POWER_SUPPLY_HEALTH_OVERVOLTAGE


# ── battery_raw_callback() tests ────────────────────────────────────────────────

class TestBatteryRawCallback:
    """battery_raw_callback() fans a [dwL, dwR, br] reading out to three BatteryState topics."""

    def test_valid_payload_publishes_three_battery_states(self, battery_node):
        msg = MagicMock()
        msg.data = [17.0, 18.0, 14.5]  # dw_l, dw_r, br

        battery_node.battery_raw_callback(msg)

        battery_node.prop_l_pub.publish.assert_called_once()
        battery_node.prop_r_pub.publish.assert_called_once()
        battery_node.main_pub.publish.assert_called_once()

        prop_l_msg = battery_node.prop_l_pub.publish.call_args[0][0]
        prop_r_msg = battery_node.prop_r_pub.publish.call_args[0][0]
        main_msg = battery_node.main_pub.publish.call_args[0][0]

        assert prop_l_msg.voltage == 17.0 and prop_l_msg.location == 'prop_l'
        assert prop_r_msg.voltage == 18.0 and prop_r_msg.location == 'prop_r'
        assert main_msg.voltage == 14.5 and main_msg.location == 'main'

    def test_short_payload_is_ignored(self, battery_node):
        msg = MagicMock()
        msg.data = [17.0, 18.0]  # only 2 values, need 3

        battery_node.battery_raw_callback(msg)

        battery_node.prop_l_pub.publish.assert_not_called()
        battery_node.prop_r_pub.publish.assert_not_called()
        battery_node.main_pub.publish.assert_not_called()
