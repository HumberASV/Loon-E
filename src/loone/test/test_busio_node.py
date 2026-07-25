from unittest.mock import MagicMock, patch

import pytest
from builtin_interfaces.msg import Time as TimeMsg


# ── Shared fixture ─────────────────────────────────────────────────────────────
#
# busio_node.py never blocks on rclpy.spin_once() during __init__ (unlike the
# old motor.py), so rclpy itself does not need to be patched here -- the real
# session-scoped ROS context from conftest.py is enough for Node.__init__,
# create_subscription/create_publisher/create_timer/get_clock to work for real.
# Only the hardware libraries (busio/board/PCA9685/INA3221/servo) are faked.
#
@pytest.fixture
def busio_node():
    busio_patcher = patch('loone.busio_node.busio')
    board_patcher = patch('loone.busio_node.board')
    pca_patcher = patch('loone.busio_node.PCA9685')
    ina_patcher = patch('loone.busio_node.INA3221')
    servo_patcher = patch('loone.busio_node.servo')

    busio_patcher.start()
    board_patcher.start()
    pca_mock = pca_patcher.start()
    ina_mock = ina_patcher.start()
    servo_mock = servo_patcher.start()

    # PCA9685(i2c, address=...) must return an object with a .channels list.
    pca_instance = MagicMock()
    pca_instance.channels = [MagicMock() for _ in range(16)]
    pca_mock.return_value = pca_instance

    # INA3221(i2c, address=..., enable=...) just needs to return *something*;
    # individual tests override node.ina directly when they need [0]/[1]/[2].
    ina_mock.return_value = MagicMock()

    # Servo(channel, min_pulse=..., max_pulse=...) is called once per channel
    # (prop_l, prop_r, rudder). A bare MagicMock call returns the same
    # `return_value` no matter the arguments, so without this side_effect all
    # three servo attributes would alias one fake object.
    servo_mock.Servo.side_effect = lambda *args, **kwargs: MagicMock()

    from loone.busio_node import BusioNode

    node = BusioNode()

    # Patch the publishers so we can inspect what gets published in tests.
    node.state_pub = MagicMock()
    node.battery_raw_pub = MagicMock()

    # Patch get_logger() so log calls inside methods don't crash.
    node.get_logger = MagicMock(return_value=MagicMock())

    yield node

    servo_patcher.stop()
    ina_patcher.stop()
    pca_patcher.stop()
    board_patcher.stop()
    busio_patcher.stop()


def _mock_clock_delta(node, delta_seconds):
    """Make node.get_clock().now() - node.last_cmd_time report a fixed age,
    without depending on real wall-clock timing."""
    fake_delta = MagicMock()
    fake_delta.nanoseconds = int(delta_seconds * 1e9)
    fake_now = MagicMock()
    fake_now.__sub__.return_value = fake_delta
    node.get_clock = MagicMock(return_value=MagicMock(now=MagicMock(return_value=fake_now)))


# ── Validation tests ───────────────────────────────────────────────────────────

class TestInitValidation:
    """Tests for frequency and pulse-range validation during initialisation."""

    def test_init_busio_freq_too_low_raises(self):
        # Fresh, unconstructed node: _init_busio must raise before completing.
        with patch('loone.busio_node.busio'), \
             patch('loone.busio_node.board'), \
             patch('loone.busio_node.PCA9685'), \
             patch('loone.busio_node.INA3221'), \
             patch('loone.busio_node.servo'):

            from loone.busio_node import BusioNode

            node = BusioNode.__new__(BusioNode)  # allocate without calling __init__
            node.get_logger = MagicMock(return_value=MagicMock())

            # freq=10 is below the minimum of 24 Hz -> should raise ValueError
            with pytest.raises(ValueError):
                node._init_busio(10)

    def test_init_busio_freq_too_high_raises(self):
        with patch('loone.busio_node.busio'), \
             patch('loone.busio_node.board'), \
             patch('loone.busio_node.PCA9685'), \
             patch('loone.busio_node.INA3221'), \
             patch('loone.busio_node.servo'):

            from loone.busio_node import BusioNode

            node = BusioNode.__new__(BusioNode)
            node.get_logger = MagicMock(return_value=MagicMock())

            with pytest.raises(ValueError):
                node._init_busio(9999)

    def test_validate_pulse_range_min_equals_max_raises(self, busio_node):
        with pytest.raises(ValueError):
            busio_node._validate_pulse_range(1500, 1500, 'test_channel')

    def test_validate_pulse_range_min_greater_than_max_raises(self, busio_node):
        with pytest.raises(ValueError):
            busio_node._validate_pulse_range(1800, 1200, 'test_channel')

    def test_validate_pulse_range_below_hardware_min_raises(self, busio_node):
        with pytest.raises(ValueError):
            busio_node._validate_pulse_range(400, 1500, 'test_channel')

    def test_validate_pulse_range_above_hardware_max_raises(self, busio_node):
        with pytest.raises(ValueError):
            busio_node._validate_pulse_range(1000, 2600, 'test_channel')

    def test_validate_pulse_range_valid_does_not_raise(self, busio_node):
        busio_node._validate_pulse_range(1120, 1880, 'prop')

    def test_init_busio_ina_failure_is_non_fatal(self):
        # A wiring fault on the INA3221 (battery monitor) must not prevent the
        # PCA9685 (propeller/rudder driver) from being set up.
        with patch('loone.busio_node.busio'), \
             patch('loone.busio_node.board'), \
             patch('loone.busio_node.PCA9685') as pca_mock, \
             patch('loone.busio_node.INA3221', side_effect=ValueError('No I2C device at address: 0x41')), \
             patch('loone.busio_node.servo'):

            pca_instance = MagicMock()
            pca_mock.return_value = pca_instance

            from loone.busio_node import BusioNode

            node = BusioNode.__new__(BusioNode)
            node.get_logger = MagicMock(return_value=MagicMock())

            node._init_busio(50)

            assert node.pca is pca_instance
            assert node.ina is None

    def test_init_busio_pca_failure_still_raises(self):
        # Unlike the INA3221, the PCA9685 is required to drive the motors at all,
        # so its failure must still be fatal.
        with patch('loone.busio_node.busio'), \
             patch('loone.busio_node.board'), \
             patch('loone.busio_node.PCA9685', side_effect=ValueError('No I2C device at address: 0x40')), \
             patch('loone.busio_node.INA3221'), \
             patch('loone.busio_node.servo'):

            from loone.busio_node import BusioNode

            node = BusioNode.__new__(BusioNode)
            node.get_logger = MagicMock(return_value=MagicMock())

            with pytest.raises(ValueError):
                node._init_busio(50)

    def test_init_servos_propagates_validation_failure(self):
        with patch('loone.busio_node.servo'):
            from loone.busio_node import BusioNode

            node = BusioNode.__new__(BusioNode)
            node.get_logger = MagicMock(return_value=MagicMock())
            node.pca = MagicMock()
            node.pca.channels = [MagicMock() for _ in range(16)]

            with pytest.raises(ValueError):
                node._init_servos(1500, 1500, 1220, 1820)  # prop min == max


# ── Command application tests ──────────────────────────────────────────────────

class TestApply:
    """_apply() writes clamped fractions to servo objects and echoes state."""

    def test_clamps_fraction_and_writes_servo(self, busio_node):
        busio_node._apply({
            'propeller_l_joint': 1.5, 'propeller_r_joint': -0.5, 'rudder_l_joint': 0.7,
        })
        assert busio_node.prop_l.fraction == 1.0
        assert busio_node.prop_r.fraction == 0.0
        assert busio_node.rudder.fraction == 0.7

    def test_publishes_state_for_applied_joints(self, busio_node):
        busio_node._apply({'propeller_l_joint': 0.8})
        busio_node.state_pub.publish.assert_called_once()
        msg = busio_node.state_pub.publish.call_args[0][0]
        assert msg.name == ['propeller_l_joint']
        # JointState.position is a float64 array.array, not a list.
        assert list(msg.position) == [0.8]

    def test_ignores_unknown_joint(self, busio_node):
        busio_node._apply({'unknown_joint': 0.5})
        busio_node.state_pub.publish.assert_called_once()
        msg = busio_node.state_pub.publish.call_args[0][0]
        assert msg.name == []

    def test_both_rudder_joints_drive_the_same_physical_servo(self, busio_node):
        # rudder_l_joint/rudder_r_joint are two independent URDF joints (one per
        # float) but there is only one physical rudder servo behind them.
        busio_node._apply({'rudder_l_joint': 0.2})
        assert busio_node.rudder.fraction == 0.2
        busio_node._apply({'rudder_r_joint': 0.9})
        assert busio_node.rudder.fraction == 0.9


class TestCommandCallback:
    """command_callback() maps a JointState command onto _apply()."""

    def test_applies_commands_from_name_position_lists(self, busio_node):
        msg = MagicMock()
        msg.name = ['propeller_l_joint', 'rudder_l_joint']
        msg.position = [0.8, 0.6]

        busio_node.command_callback(msg)

        assert busio_node.prop_l.fraction == 0.8
        assert busio_node.rudder.fraction == 0.6

    def test_advances_last_cmd_time(self, busio_node):
        sentinel_time = MagicMock()
        # _apply() -> _publish_state() also calls .to_msg() on this same mocked
        # now() for the JointState header stamp, and the message setter asserts
        # a real Time type -- give it one instead of a bare MagicMock.
        sentinel_time.to_msg.return_value = TimeMsg()
        busio_node.get_clock = MagicMock(
            return_value=MagicMock(now=MagicMock(return_value=sentinel_time)))

        msg = MagicMock()
        msg.name = ['propeller_l_joint']
        msg.position = [0.5]
        busio_node.command_callback(msg)

        assert busio_node.last_cmd_time is sentinel_time

    def test_ignores_command_when_position_shorter_than_name(self, busio_node):
        msg = MagicMock()
        msg.name = ['propeller_l_joint', 'propeller_r_joint', 'rudder_l_joint']
        msg.position = [0.8]  # shorter than name list

        busio_node._apply = MagicMock()
        busio_node.command_callback(msg)

        busio_node._apply.assert_not_called()


# ── Watchdog tests ──────────────────────────────────────────────────────────────

class TestWatchdog:
    """watchdog() is the dead-man switch: force neutral once commands go stale."""

    def test_holds_neutral_when_stale(self, busio_node):
        _mock_clock_delta(busio_node, delta_seconds=1.0)  # > default cmd_timeout (0.5s)
        busio_node._apply = MagicMock()

        busio_node.watchdog()

        busio_node._apply.assert_called_once_with(dict(busio_node.joint_neutral))

    def test_does_nothing_when_fresh(self, busio_node):
        _mock_clock_delta(busio_node, delta_seconds=0.1)  # < default cmd_timeout (0.5s)
        busio_node._apply = MagicMock()

        busio_node.watchdog()

        busio_node._apply.assert_not_called()


# ── Battery telemetry tests ─────────────────────────────────────────────────────

class TestPublishBatteryRaw:
    """publish_battery_raw() forwards the three raw INA3221 bus voltages, unconverted."""

    def test_publishes_raw_ina_voltages_in_order(self, busio_node):
        busio_node.ina = [
            MagicMock(bus_voltage=11.1),
            MagicMock(bus_voltage=12.2),
            MagicMock(bus_voltage=13.3),
        ]

        busio_node.publish_battery_raw()

        busio_node.battery_raw_pub.publish.assert_called_once()
        msg = busio_node.battery_raw_pub.publish.call_args[0][0]
        # Float32MultiArray.data is a float32 array.array, not a list -- convert
        # before comparing, and use approx for the float32 rounding.
        assert list(msg.data) == pytest.approx([11.1, 12.2, 13.3])

    def test_skips_publishing_when_ina_unavailable(self, busio_node):
        # self.ina is None when the INA3221 failed to initialize (see
        # test_init_busio_ina_failure_is_non_fatal) -- publishing must no-op
        # rather than crash on `self.ina[0]`.
        busio_node.ina = None

        busio_node.publish_battery_raw()

        busio_node.battery_raw_pub.publish.assert_not_called()


# ── Shutdown tests ──────────────────────────────────────────────────────────────

class TestShutdown:

    def test_applies_neutral_then_deinits(self, busio_node):
        busio_node._apply = MagicMock()

        busio_node.shutdown()

        busio_node._apply.assert_called_once_with(dict(busio_node.joint_neutral))
        busio_node.pca.deinit.assert_called_once()

    def test_deinits_even_if_apply_raises(self, busio_node):
        # shutdown()'s try/finally must still release the I2C bus on failure.
        busio_node._apply = MagicMock(side_effect=RuntimeError('boom'))

        with pytest.raises(RuntimeError):
            busio_node.shutdown()

        busio_node.pca.deinit.assert_called_once()
