"""
Hobby servos (the gripper), 270 degree type, on the schematic's ServoPin1 /
ServoPin2 (config.SERVO_PINS). Moved here from the old servos.py, without
the test loop that used to run as soon as the file was imported (the test is
now tests/test_servos.py).

A servo is told its angle by the length of a pulse, sent 50 times a second:
    0.5 ms -> 0 degrees,  1.5 ms -> 135 degrees,  2.5 ms -> 270 degrees.
The Pico's PWM duty is a fraction of 65535, and one period is 20 ms, so
    0.5 ms = 0.025 of the period -> duty  1638
    2.5 ms = 0.125 of the period -> duty  8192
Many servos do not reach exactly 0 / 270 degrees with these pulses; adjust
MIN_DUTY / MAX_DUTY for yours if the end positions are off.

FIX (2026-10-23): NEVER more than 270 degrees (2.5 ms, the servo's longest
valid pulse). For a short time angles up to 300 (2.72 ms) were allowed; at
300 servo2 ran away and kept turning faster. ANGLE_LIMIT = 270 is a hard
ceiling in _drive(), whatever config.SERVO_SAFE_RANGE says.

The servo holds the angle it was given (it has its own feedback inside),
but the Pico cannot read the real angle back: angle() only returns the last
angle that was set.

Usage
    from actuators.servo import Servo
    import config
    gripper = Servo(config.SERVO_PINS["servo2"])   # safe range / reset path from config.py
    gripper.reset()             # REQUIRED first move after power-up: 222 -> 250, slowly
    gripper.set_angle(240)      # always kept inside the safe range (222 - 270)
"""

from machine import Pin, PWM
from utime import sleep_ms


class Servo:
    MIN_DUTY = 1638     # 0.5 ms pulse = 0 degrees
    MAX_DUTY = 8192     # 2.5 ms pulse = 270 degrees
    MAX_ANGLE = 270     # the scale: MAX_DUTY is this angle
    ANGLE_LIMIT = 270   # FIX (2026-10-23): never send more (pulses over 2.5 ms make the servo run away)

    def __init__(self, pin_number, safe_range=None, reset_path=None):
        self._pwm = PWM(Pin(pin_number))
        self._pwm.freq(50)              # 50 Hz = one pulse every 20 ms
        self._angle = None
        # ADDED (2026-10-23): safe_range = (lowest, highest) angle this servo
        # may be sent to (config.SERVO_SAFE_RANGE). Beyond it the gripper hits
        # its end stop and the stalled servo can reset the Pico.
        # reset_path = (start, end) used by reset() (config.SERVO_RESET).
        # When they are not given, they are looked up in config.py by the pin
        # number, so even code that only writes Servo(15) is protected.
        name = self._config_name(pin_number)
        if safe_range is None:
            safe_range = self._from_config("SERVO_SAFE_RANGE", name, (0, self.MAX_ANGLE))
        if reset_path is None:
            reset_path = self._from_config("SERVO_RESET", name, None)
        self.low, self.high = safe_range
        self.reset_path = reset_path

    @staticmethod
    def _config_name(pin_number):
        """ADDED (2026-10-23): "servo1" / "servo2" for a pin in config.SERVO_PINS, else None."""
        try:
            import config
            for name, pin in config.SERVO_PINS.items():
                if pin == pin_number:
                    return name
        except (ImportError, AttributeError):
            pass
        return None

    @staticmethod
    def _from_config(setting, name, default):
        """ADDED (2026-10-23): config.<setting>[name] if it exists, else default."""
        try:
            import config
            value = getattr(config, setting).get(name)
        except (ImportError, AttributeError):
            value = None
        return default if value is None else value

    def reset(self, settle_ms=600, step_ms=20):
        """ADDED (2026-10-23): bring the servo to its start position the safe way.

        With a reset_path (start, end): go to `start`, wait settle_ms so the
        servo has really arrived there, then turn slowly to `end`, one degree
        every step_ms. So the servo always finishes by moving from start to
        end (servo2: 222 -> 250, the only direction that is safe for the
        gripper). Without a reset_path: straight to the middle of the safe
        range. Every angle sent is inside the safe range. This WAITS until
        the servo is there (about 1.6 s for servo2). Returns the final angle."""
        if self.reset_path is None:
            self._drive((self.low + self.high) // 2)
            return self._angle
        start, end = self.reset_path
        self._drive(start)
        sleep_ms(settle_ms)
        direction = 1 if end >= start else -1
        for angle in range(start + direction, end + direction, direction):
            self._drive(angle)
            sleep_ms(step_ms)
        return self._angle

    def set_angle(self, angle):
        """Move to `angle` degrees (clamped to the safe range, see __init__).

        CHANGED (2026-10-23): a servo with no pulses (just created, or after
        off()) is NOT moved: its real position is unknown, and jumping
        straight to an angle could turn it the unsafe way. Call reset() first.
        Returns True if the servo was moved."""
        if self._angle is None:
            print("servo (safe range %d-%d) has no pulses yet: set_angle(%s) ignored, call reset() first"
                  % (self.low, self.high, angle))
            return False
        self._drive(angle)
        return True

    def _drive(self, angle):
        """ADDED (2026-10-23): send the pulse for `angle`, clamped into the safe
        range (the old set_angle body; it clamped to 0 .. 270)."""
        if angle < self.low:
            angle = self.low
        if angle > self.high:
            angle = self.high
        if angle > self.ANGLE_LIMIT:      # ADDED (2026-10-23): hard ceiling, whatever the range says
            angle = self.ANGLE_LIMIT
        if angle < 0:
            angle = 0
        duty = int(self.MIN_DUTY + angle / self.MAX_ANGLE * (self.MAX_DUTY - self.MIN_DUTY))
        self._pwm.duty_u16(duty)
        self._angle = angle

    def angle(self):
        """The last angle set (None before the first set_angle)."""
        return self._angle

    def off(self):
        """Stop sending pulses: the servo goes limp."""
        self._pwm.duty_u16(0)
        self._angle = None
