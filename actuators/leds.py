"""
The three indicator LEDs (yellow, red, green).

They are switched through a 4-channel optocoupler (220 ohm on both sides), so
a GPIO set HIGH turns its LED on. Pins come from config.LED_PINS.

Usage
    from actuators.leds import StatusLeds
    leds = StatusLeds.from_config()
    leds.set("green", True)
    leds.show(green=True, red=False, yellow=False)
    leds.off()
"""

from machine import Pin


class StatusLeds:
    """pins: {"yellow": GP, "red": GP, "green": GP}"""

    def __init__(self, pins):
        self._pins = {}
        for colour, gp in pins.items():
            self._pins[colour] = Pin(gp, Pin.OUT, value=0)

    @classmethod
    def from_config(cls):
        import config
        return cls(config.LED_PINS)

    def set(self, colour, on):
        pin = self._pins.get(colour)
        if pin is not None:
            pin.value(1 if on else 0)

    def show(self, **states):
        """show(green=True, red=False, ...) - only the colours given are changed."""
        for colour, on in states.items():
            self.set(colour, on)

    def off(self):
        for pin in self._pins.values():
            pin.value(0)

    def is_on(self, colour):
        pin = self._pins.get(colour)
        return pin is not None and pin.value() == 1
