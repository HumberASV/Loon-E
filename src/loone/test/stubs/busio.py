# Placeholder for Adafruit Blinka's `busio` module, used only so that
# `import busio` succeeds in CI (no real SBC hardware / I2C bus present).
# motor.py only ever touches this module through calls that the test
# suite patches with MagicMock before they run, so no real behavior
# is needed here. Never shadows the real Blinka `busio` on robot hardware,
# since this directory is only added to PYTHONPATH for CI test runs.


class I2C:
    def __init__(self, *args, **kwargs):
        pass


# adafruit_pca9685 transitively imports adafruit_bus_device.spi_device,
# whose module-level `from busio import SPI` must succeed (its except
# clause only resets DigitalInOut, never defines SPI), since SPIDevice's
# __init__ uses a bare (eagerly-evaluated) `spi: SPI` annotation.
class SPI:
    def __init__(self, *args, **kwargs):
        pass
