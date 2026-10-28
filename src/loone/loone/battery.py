from typing import Protocol


class INA3221Channel(Protocol):
    """INA3221 channel measurements used to calculate battery-side voltage."""

    @property
    def bus_voltage(self) -> float:
        """Voltage at the load side of the shunt, in volts."""

    @property
    def shunt_voltage(self) -> float:
        """Voltage drop across the shunt, in millivolts."""


def battery_voltage(channel: INA3221Channel) -> float:
    """Return battery-side voltage when the battery is wired to VIN+."""
    return channel.bus_voltage + channel.shunt_voltage / 1000.0
