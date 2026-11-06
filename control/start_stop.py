from utime import ticks_ms

from sensors.crash import CrashSensor


class StartStopButton:
    def __init__(self, sensor):
        self.sensor = sensor

    @classmethod
    def from_config(cls):
        import config
        if config.START_STOP_SENSOR == "push":
            from sensors.push_button import PushButton
            return cls(PushButton(config.PUSH_BUTTON_PIN, config.CRASH_ACTIVE_LOW, config.CRASH_DEBOUNCE_MS,
                                  name="push button (start/stop)"))
        pin = config.CRASH_LEFT_PIN if config.START_STOP_SENSOR == "left" else config.CRASH_RIGHT_PIN
        return cls(CrashSensor(pin, config.CRASH_ACTIVE_LOW, config.CRASH_DEBOUNCE_MS,
                               name=config.START_STOP_SENSOR + " crash (start/stop)"))

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
