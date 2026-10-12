"""
Hardware interface for the 4 x DFRobot SEN0017 line tracking sensors.

Each SEN0017 has one digital (TTL) output: HIGH over white, LOW over black
(DFRobot wiki). Supply 3.3-5 V; power them from the Pico 3V3 so the output can
go straight to a GPIO. Mount them 1-2 cm above the floor.

Wiring: VCC -> 3V3, GND -> GND, signal -> the GP pins in config.py
(LINE_OUTER_LEFT_PIN, LINE_INNER_LEFT_PIN, LINE_INNER_RIGHT_PIN,
LINE_OUTER_RIGHT_PIN), ordered left to right across the robot.

read() returns the 4-bit mask used by sensors/line_processing.py
(bit set = white = line). All processing (steering error, junctions, turns)
lives in line_processing.py so it can be tested without hardware.
"""

from machine import Pin

from sensors.line_processing import (OUTER_LEFT, INNER_LEFT, INNER_RIGHT,
                                     OUTER_RIGHT, bits_to_str)

_MASKS = (OUTER_LEFT, INNER_LEFT, INNER_RIGHT, OUTER_RIGHT)


class LineSensorArray:
    """Four line sensors read as one bitmask.

    pins: 4 GPIO numbers ordered outer-left, inner-left, inner-right, outer-right.
    white_is_high: True for the SEN0017 (white -> HIGH).
    samples: reads per pin per read(); a majority vote rejects single glitches.
    pull: optional Pin.PULL_UP / Pin.PULL_DOWN (None = no pull; the sensor drives the line).
    """

    def __init__(self, pins, white_is_high=True, samples=3, pull=None):
        if len(pins) != 4:
            raise ValueError("need 4 pins: outer-left, inner-left, inner-right, outer-right")
        if samples < 1:
            raise ValueError("samples must be >= 1")
        if pull is None:
            self._pins = [Pin(p, Pin.IN) for p in pins]
        else:
            self._pins = [Pin(p, Pin.IN, pull) for p in pins]
        self.pin_numbers = tuple(pins)
        self.white_is_high = white_is_high
        self.samples = samples
        self._majority = samples // 2 + 1
        self.bits = 0

    @classmethod
    def from_config(cls):
        import config
        return cls((config.LINE_OUTER_LEFT_PIN, config.LINE_INNER_LEFT_PIN,
                    config.LINE_INNER_RIGHT_PIN, config.LINE_OUTER_RIGHT_PIN),
                   white_is_high=config.LINE_WHITE_IS_HIGH,
                   samples=config.LINE_SAMPLES_PER_READ)

    def read(self):
        """Return the 4-bit mask (bit set = sensor sees white). Non-blocking."""
        white_level = 1 if self.white_is_high else 0
        bits = 0
        for i in range(4):
            pin = self._pins[i]
            votes = 0
            for _ in range(self.samples):
                if pin.value() == white_level:
                    votes += 1
            if votes >= self._majority:
                bits |= _MASKS[i]
        self.bits = bits
        return bits

    def read_raw(self):
        """Raw pin levels left to right (no inversion, no voting), for diagnostics."""
        return [pin.value() for pin in self._pins]

    def __str__(self):
        return bits_to_str(self.bits)
