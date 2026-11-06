from machine import Pin, PWM
from utime import sleep_ms


class Servo:
    MIN_DUTY = 1638     # 0.5 ms pulse = 0 degrees
    MAX_DUTY = 8192     # 2.5 ms pulse = 270 degrees
    MAX_ANGLE = 270     # the scale: MAX_DUTY is this angle
    ANGLE_LIMIT = 270   # never send more (pulses over 2.5 ms make the servo run away)

    def __init__(self, pin_number, safe_range=None, reset_path=None):
        self._pwm = PWM(Pin(pin_number))
        self._pwm.freq(50)              # 50 Hz = one pulse every 20 ms
        self._angle = None
        name = self._config_name(pin_number)
        self.blocked = False
        if safe_range is None:
            safe_range = self._from_config("SERVO_SAFE_RANGE", name, None)
            if safe_range is None:
                self.blocked = name is not None     # only servos that config.py knows
                safe_range = (0, int(self.MAX_ANGLE/2),  self.MAX_ANGLE)
        if reset_path is None:
            reset_path = self._from_config("SERVO_RESET", name, None)
        self.low, self.medium, self.high = safe_range
        self.reset_path = reset_path

    @staticmethod
    def _config_name(pin_number):
        """"servo1" / "servo2" for a pin in config.SERVO_PINS, else None."""
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
        """Config.<setting>[name] if it exists, else default."""
        try:
            import config
            value = getattr(config, setting).get(name)
        except (ImportError, AttributeError):
            value = None
        return default if value is None else value

    def reset(self, settle_ms=600, step_ms=20, target=None):
        """Bring the servo to its start position the safe way.

        With a reset_path (start, end): go to `start`, wait settle_ms so the
        servo has really arrived there, then turn slowly to `end`, one degree
        every step_ms. So the servo always finishes by moving from start to
        end (for a gripper where only one direction is safe). Without a
        reset_path: straight to the middle of the safe range, then wait
        settle_ms. Every angle sent
        is inside the safe range. This WAITS until the servo is there
        (settle_ms, plus the slow turn if there is a path). Returns the
        final angle.

        target: without a reset_path, go straight to this
        angle instead of the middle (still clamped into the safe range).
        gripper_start() passes the carry pose, so the gripper makes ONE move
        at start-up instead of middle-then-carry (the arm used to dip down to
        the middle and then lift again)."""
        if self.reset_path is None:
            if target is None:
                target = (self.low + self.high) // 2
            self._drive(target)                 # _drive clamps into the safe range
            sleep_ms(settle_ms)
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

        A servo with no pulses (just created, or after
        off()) is NOT moved: its real position is unknown, and jumping
        straight to an angle could turn it the unsafe way. Call reset() first.
        Returns True if the servo was moved."""
        if self._angle is None:
            print("servo (safe range %d-%d) has no pulses yet: set_angle(%s) ignored, call reset() first"
                  % (self.low, self.high, angle))
            return False
        self._drive(angle)
        return True

    def move_slowly(self, angle, step_ms=10, low=None):
        """Turn to `angle` one degree at a time, waiting
        step_ms after each degree, so the gripper moves gently instead of
        jumping. The target is clamped into the safe range first, and every
        step lies between the present angle and the target, so the servo
        only moves inside its safe range and only in one direction.
        Like set_angle(), it does nothing on a servo with no pulses (call
        reset() first). WAITS until the end. Returns True if it moved.

        low: a different lowest angle for THIS move only,
        instead of the safe range's low end. Used for ONE thing: closing the
        gripper further than its normal range to push a box into its slot
        (config.SERVO_PUSH_ANGLE), and opening it again
        from there. Every other move keeps the normal safe range."""
        if self._angle is None:
            print("servo (safe range %d-%d) has no pulses yet: move_slowly(%s) ignored, call reset() first"
                  % (self.low, self.high, angle))
            return False
        floor = self.low if low is None else max(0, low)
        target = max(floor, min(self.high, int(round(angle))))
        current = int(round(self._angle))
        direction = 1 if target >= current else -1
        for step in range(current + direction, target + direction, direction):
            self._drive(step, floor)
            sleep_ms(step_ms)
        self._drive(target, floor)
        return True

    def start_at(self, angle):
        """CALIBRATION ONLY. Start a servo straight at
        `angle` (still clamped into the safe range and to 270), even though
        it has no pulses yet. Used by the sensor viewer's calibration mode
        after the gripper hardware changed, when no safe reset is known.
        The robot code must use reset() instead. Returns the angle sent."""
        self._drive(angle)
        return self._angle

    def _drive(self, angle, low=None):
        """Send the pulse for `angle`, clamped into the safe
        range.
        low: only move_slowly(..., low=...) passes it
        (the push at the drop-off); otherwise the safe range's low end."""
        if self.blocked:                  # no safe range known -> never move
            print("servo on a pin without a safe range in config.SERVO_SAFE_RANGE: not moved")
            return
        floor = self.low if low is None else low
        if angle < floor:
            angle = floor
        if angle > self.high:
            angle = self.high
        if angle > self.ANGLE_LIMIT:      # hard ceiling, whatever the range says
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
