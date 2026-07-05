# Copyright 2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
This is a standard Unit test for Motor Node.
"""


# unittest.mock is a built-in Python library for replacing real objects with
# fakes during tests so that hardware / network / OS calls never actually run.
#
#   patch(target)  – temporarily swaps the named object for a fake.
#                    "target" is a dotted path: 'loone.motor.busio' means
#                    "the name 'busio' as it exists inside loone/motor.py".
#
#   MagicMock()    – a fake object that accepts ANY attribute access or method
#                    call without crashing, and records every interaction so
#                    you can assert on them later.
#
#   call(...)      – a helper used with assert_called_with / assert_any_call
#                    to describe what arguments a mock was expected to receive.

from unittest.mock import MagicMock, patch, call  # noqa: F401
import numpy as np
import pytest


# ── Shared fixture ─────────────────────────────────────────────────────────────
#
# A pytest fixture is a function decorated with @pytest.fixture.
# Any test that lists `motor_node` as a parameter automatically receives the
# Motor instance built here — you never call the fixture yourself.
#
# We use `yield` so the fixture has two phases:
#   1. everything before `yield`  → setup (runs before the test)
#   2. everything after  `yield`  → teardown (runs after the test, even if it fails)
#
@pytest.fixture
def motor_node():
    # patch() returns a "patcher" object.  patcher.start() activates the swap
    # and returns the MagicMock that now sits in place of the real thing.
    # We MUST call patcher.stop() when the test finishes, otherwise the swap
    # leaks into other tests.

    # Replace the entire rclpy module so Node.__init__ never tries to talk to
    # a running ROS2 daemon.
    rclpy_patcher = patch('loone.motor.rclpy')

    # Replace busio so I2C() never tries to open a real I2C bus.
    busio_patcher = patch('loone.motor.busio')

    # Replace board so board.SCL / board.SDA don't require GPIO pins.
    board_patcher = patch('loone.motor.board')

    # Replace PCA9685 so the chip is never physically addressed.
    pca_patcher = patch('loone.motor.PCA9685')

    # Replace the servo module so Servo() objects are fakes we can inspect.
    servo_patcher = patch('loone.motor.servo')

    rclpy_patcher.start()
    busio_patcher.start()
    board_patcher.start()
    pca_mock = pca_patcher.start()
    servo_patcher.start()

    # PCA9685(i2c) is called with the i2c bus as argument and must return an
    # object with a .channels list.  We build a fake PCA9685 instance here.
    pca_instance = MagicMock()
    pca_instance.channels = [MagicMock() for _ in range(16)]
    pca_mock.return_value = pca_instance  # PCA9685(...) now returns pca_instance

    # Motor also calls self.create_subscription / create_publisher / get_logger,
    # which are real Node methods (conftest.py's session-scoped rclpy.init()
    # keeps the default context valid, so Node.__init__ runs for real here).
    from loone.motor import Motor

    node = Motor()

    # Patch the publisher so we can check what gets published in drive() tests.
    node.publisher_ = MagicMock()

    # Patch get_logger() so log calls inside methods don't crash.
    node.get_logger = MagicMock(return_value=MagicMock())

    yield node  # ← the test runs here, receiving `node`

    # Teardown: restore every patched name to its original value.
    rclpy_patcher.stop()
    busio_patcher.stop()
    board_patcher.stop()
    pca_patcher.stop()
    servo_patcher.stop()


# ── Validation tests ───────────────────────────────────────────────────────────

class TestInitValidation:
    """Tests for frequency and pulse-range validation during initialisation."""

    def test_invalid_pca_freq_too_low_raises(self):
        # We need a fresh Motor for this, but we want _init_pca to raise before
        # completing.  We still need all the hardware patches active.
        with patch('loone.motor.rclpy'), \
             patch('loone.motor.busio'), \
             patch('loone.motor.board'), \
             patch('loone.motor.PCA9685'), \
             patch('loone.motor.servo'):

            from loone.motor import Motor
            node = Motor.__new__(Motor)  # allocate without calling __init__
            node.get_logger = MagicMock(return_value=MagicMock())

            # freq=10 is below the minimum of 24 Hz → should raise ValueError
            with pytest.raises(ValueError):
                node._init_pca(10)

    def test_invalid_pca_freq_too_high_raises(self):
        with patch('loone.motor.rclpy'), \
             patch('loone.motor.busio'), \
             patch('loone.motor.board'), \
             patch('loone.motor.PCA9685'), \
             patch('loone.motor.servo'):

            from loone.motor import Motor
            node = Motor.__new__(Motor)
            node.get_logger = MagicMock(return_value=MagicMock())

            with pytest.raises(ValueError):
                node._init_pca(9999)

    def test_validate_pulse_range_min_equals_max_raises(self, motor_node):
        # min_pulse == max_pulse is invalid (no range to work with)
        with pytest.raises(ValueError):
            motor_node._validate_pulse_range(1500, 1500, 'test_channel')

    def test_validate_pulse_range_min_greater_than_max_raises(self, motor_node):
        with pytest.raises(ValueError):
            motor_node._validate_pulse_range(1800, 1200, 'test_channel')

    def test_validate_pulse_range_below_hardware_min_raises(self, motor_node):
        # 400 µs is below the hardware limit of 500 µs
        with pytest.raises(ValueError):
            motor_node._validate_pulse_range(400, 1500, 'test_channel')

    def test_validate_pulse_range_above_hardware_max_raises(self, motor_node):
        # 2600 µs exceeds the hardware limit of 2500 µs
        with pytest.raises(ValueError):
            motor_node._validate_pulse_range(1000, 2600, 'test_channel')

    def test_validate_pulse_range_valid_does_not_raise(self, motor_node):
        # Should complete without raising anything
        motor_node._validate_pulse_range(1120, 1880, 'prop')


# ── Pure math tests ────────────────────────────────────────────────────────────

class TestConvert:
    """convert() maps [0, 360] heading to [-180, 180]."""

    def test_angle_below_180_unchanged(self, motor_node):
        assert motor_node.convert(90) == 90

    def test_angle_exactly_180_unchanged(self, motor_node):
        assert motor_node.convert(180) == 180

    def test_angle_above_180_wraps(self, motor_node):
        # 270° clockwise == -90° (turn left 90°)
        assert motor_node.convert(270) == -90

    def test_angle_360_becomes_0(self, motor_node):
        assert motor_node.convert(360) == 0


class TestRemap:
    """remap() maps a heading error to a pulse width in microseconds."""

    def test_zero_error_returns_outmax(self, motor_node):
        # Zero error → no correction → maximum pulse (full speed)
        result = motor_node.remap(0, outMin=1540, outMax=1880)
        assert result == 1880

    def test_max_error_returns_outmin(self, motor_node):
        # Full error (== self.max) → maximum correction → minimum pulse
        result = motor_node.remap(motor_node.max, outMin=1540, outMax=1880)
        assert result == 1540

    def test_half_error_is_midpoint(self, motor_node):
        # Half error → half correction → midpoint between outMin and outMax
        result = motor_node.remap(motor_node.max / 2, outMin=1540, outMax=1880)
        assert result == pytest.approx(1710, rel=1e-3)


class TestGetFraction:
    """get_fraction() converts a pulse width µs → [0.0, 1.0] duty cycle."""

    def test_min_pulse_returns_zero(self, motor_node):
        assert motor_node.get_fraction(1120, min_pulse=1120, max_pulse=1880) == 0.0

    def test_max_pulse_returns_one(self, motor_node):
        assert motor_node.get_fraction(1880, min_pulse=1120, max_pulse=1880) == 1.0

    def test_midpoint_returns_half(self, motor_node):
        result = motor_node.get_fraction(1500, min_pulse=1120, max_pulse=1880)
        assert result == pytest.approx(0.5, rel=1e-3)

    def test_below_min_clamps_to_zero(self, motor_node):
        # Pulse below range → clamp to 0.0 (don't crash, don't go negative)
        result = motor_node.get_fraction(1000, min_pulse=1120, max_pulse=1880)
        assert result == 0.0

    def test_above_max_clamps_to_one(self, motor_node):
        result = motor_node.get_fraction(2000, min_pulse=1120, max_pulse=1880)
        assert result == 1.0

    def test_invalid_range_raises(self, motor_node):
        with pytest.raises(ValueError):
            motor_node.get_fraction(1500, min_pulse=1880, max_pulse=1120)


# ── State check tests ──────────────────────────────────────────────────────────

class TestCheckData:
    """check_data() returns True only when all four values are ready."""

    def test_returns_false_when_heading_is_nan(self, motor_node):
        motor_node.current_heading = np.nan
        motor_node.current_speed = 1.0
        motor_node.target_heading = 90.0
        motor_node.target_speed = 1.0
        assert motor_node.check_data() is False

    def test_returns_false_when_speed_is_nan(self, motor_node):
        motor_node.current_heading = 90.0
        motor_node.current_speed = np.nan
        motor_node.target_heading = 90.0
        motor_node.target_speed = 1.0
        assert motor_node.check_data() is False

    def test_returns_false_when_target_heading_is_none(self, motor_node):
        motor_node.current_heading = 90.0
        motor_node.current_speed = 1.0
        motor_node.target_heading = None
        motor_node.target_speed = 1.0
        assert motor_node.check_data() is False

    def test_returns_false_when_target_speed_is_none(self, motor_node):
        motor_node.current_heading = 90.0
        motor_node.current_speed = 1.0
        motor_node.target_heading = 90.0
        motor_node.target_speed = None
        assert motor_node.check_data() is False

    def test_returns_true_when_all_data_present(self, motor_node):
        motor_node.current_heading = 90.0
        motor_node.current_speed = 1.0
        motor_node.target_heading = 90.0
        motor_node.target_speed = 1.0
        assert motor_node.check_data() is True


# ── Callback tests ─────────────────────────────────────────────────────────────

class TestCallbacks:

    def test_phone_callback_stores_speed_and_heading(self, motor_node):
        # Build a fake ROS message.  Float32MultiArray.data is index-addressable.
        msg = MagicMock()
        msg.data = [0.0, 0.0, 2.5, 135.0]  # index 2 = speed, 3 = heading

        motor_node.phone_callback(msg)

        assert motor_node.current_speed == 2.5
        assert motor_node.current_heading == 135.0

    def test_task_callback_stores_heading_and_speed(self, motor_node):
        msg = MagicMock()
        msg.data = [0.0, 45.0, 1.5]  # index 1 = target heading, 2 = target speed

        # Keep sensor data invalid so drive() is NOT triggered yet
        motor_node.current_heading = np.nan

        motor_node.task_callback(msg)

        assert motor_node.target_heading == 45.0
        assert motor_node.target_speed == 1.5

    def test_task_callback_does_not_drive_when_data_invalid(self, motor_node):
        msg = MagicMock()
        msg.data = [0.0, 45.0, 1.5]
        motor_node.current_heading = np.nan  # data still invalid

        # Swap drive() for a MagicMock so we can check it was NOT called
        motor_node.drive = MagicMock()
        motor_node.task_callback(msg)

        motor_node.drive.assert_not_called()

    def test_task_callback_drives_when_data_valid(self, motor_node):
        motor_node.current_heading = 90.0
        motor_node.current_speed = 1.0

        msg = MagicMock()
        msg.data = [0.0, 45.0, 1.5]

        motor_node.drive = MagicMock()
        motor_node.task_callback(msg)

        # drive() must be called exactly once when all data is present
        motor_node.drive.assert_called_once()


# ── Drive / PID tests ──────────────────────────────────────────────────────────

class TestDrive:
    """Tests for the PID control loop in drive()."""

    def _setup(self, motor_node, current_heading, target_heading,
               current_speed=1.0, target_speed=1.0):
        """Helper: configure state then call drive() once."""
        motor_node.current_heading = current_heading
        motor_node.target_heading = target_heading
        motor_node.current_speed = current_speed
        motor_node.target_speed = target_speed
        motor_node.drive()

    def test_drive_publishes_motor_state(self, motor_node):
        self._setup(motor_node, current_heading=0.0, target_heading=10.0)
        # publisher_.publish must be called exactly once per drive() cycle
        motor_node.publisher_.publish.assert_called_once()

    def test_positive_error_reduces_right_propeller(self, motor_node):
        # current_error = 10 - 0 = +10  →  turn right: prop_r fraction < factor
        self._setup(motor_node, current_heading=0.0, target_heading=10.0)
        assert motor_node.prop_l.fraction == motor_node.factor
        assert motor_node.prop_r.fraction < motor_node.factor

    def test_negative_error_reduces_left_propeller(self, motor_node):
        # current_error = -10  →  turn left: prop_l fraction < factor
        self._setup(motor_node, current_heading=10.0, target_heading=0.0)
        assert motor_node.prop_r.fraction == motor_node.factor
        assert motor_node.prop_l.fraction < motor_node.factor

    def test_rudder_turns_right_on_large_negative_output(self, motor_node):
        # A large leftward error produces a large negative PID output → rudder = 0
        motor_node.kp = 1
        self._setup(motor_node, current_heading=45.0, target_heading=0.0)
        # Only check if error exceeds half of max (45/2 = 22.5)
        if motor_node.last_error < -(motor_node.max / 2):
            assert motor_node.rudder.fraction == 0

    def test_rudder_centred_on_small_output(self, motor_node):
        # A tiny error keeps the rudder centred at fraction 0.55
        motor_node.kp = 0  # zero gain → output is always 0
        self._setup(motor_node, current_heading=0.0, target_heading=1.0)
        assert motor_node.rudder.fraction == 0.55

    def test_speed_factor_increases_when_below_target(self, motor_node):
        initial_factor = motor_node.factor
        self._setup(motor_node, current_heading=0.0, target_heading=0.0,
                    current_speed=0.5, target_speed=1.0)
        assert motor_node.factor > initial_factor

    def test_speed_factor_decreases_when_above_target(self, motor_node):
        motor_node.factor = 0.9
        self._setup(motor_node, current_heading=0.0, target_heading=0.0,
                    current_speed=2.0, target_speed=1.0)
        assert motor_node.factor < 0.9

    def test_speed_factor_floor_is_0_55(self, motor_node):
        motor_node.factor = 0.55  # already at floor
        self._setup(motor_node, current_heading=0.0, target_heading=0.0,
                    current_speed=2.0, target_speed=1.0)
        assert motor_node.factor >= 0.55


# ── Shutdown test ──────────────────────────────────────────────────────────────

class TestShutdown:

    def test_shutdown_calls_pca_deinit(self, motor_node):
        motor_node.shutdown()
        # pca.deinit() must be called exactly once to release the I2C bus
        motor_node.pca.deinit.assert_called_once()
