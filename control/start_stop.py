"""
A crash sensor used as the start / stop button.

Each press (debounced, short taps included) is reported once by pressed().
Mounted at the front, the same switch also stops the robot when it bumps
into something. Which sensor is used: config.START_STOP_SENSOR.

Usage
    from control.start_stop import StartStopButton
    button = StartStopButton.from_config()
    while True:
        if button.pressed():
            ...toggle run / stop...
"""

from utime import ticks_ms

from sensors.crash import CrashSensor
from sensors.push_button import PushButton
from config import PUSH_BUTTON_PIN
from machine import Pin







class StartStopButton:
    def __init__(self, sensor):
        self.sensor = sensor

    @classmethod
    def from_config(cls):
        import config
        if config.START_STOP_SENSOR == "left":
            pin = config.CRASH_LEFT_PIN
            return cls(CrashSensor(pin, config.CRASH_ACTIVE_LOW, config.CRASH_DEBOUNCE_MS,
                               name=config.START_STOP_SENSOR + " crash (start/stop)"))

        elif config.START_STOP_SENSOR == "right":
            pin = config.CRASH_RIGHT_PIN
            return cls(CrashSensor(pin, config.CRASH_ACTIVE_LOW, config.CRASH_DEBOUNCE_MS,
                               name=config.START_STOP_SENSOR + " crash (start/stop)"))

        else:
            pin = config.PUSH_BUTTON_PIN
            return cls(PushButton(PUSH_BUTTON_PIN, config.CRASH_ACTIVE_LOW))
        



    def pressed(self, now_ms=None):
        """Call every loop. True once for every new press."""
        if now_ms is None:
            now_ms = ticks_ms()
        self.sensor.update(now_ms)
        return self.sensor.consume_hit()

    def held(self):
        """True while the switch is held down (debounced)."""
        return self.sensor.pressed

    def deinit(self):
        self.sensor.deinit()
