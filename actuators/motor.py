from machine import Pin, PWM


def _clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


class Motor:
    """One motor: dir_pin / pwm_pin GPIO numbers."""

    def __init__(self, dir_pin, pwm_pin, invert=False, freq=1000, min_duty=0):
        if not 0 <= min_duty < 100:
            raise ValueError("min_duty must be 0 .. 99")
        self.invert = invert
        self.min_duty = min_duty
        self._dir = Pin(dir_pin, Pin.OUT, value=0)
        self._pwm = PWM(Pin(pwm_pin))
        self._pwm.freq(freq)
        self._pwm.duty_u16(0)
        self.speed = 0

    def set(self, speed):
        """Set the speed in percent, -100 .. +100 (clamped)."""
        speed = _clamp(speed, -100, 100)
        self.speed = speed
        magnitude = abs(speed)
        if magnitude < 0.5:
            self._pwm.duty_u16(0)
            return
        duty = self.min_duty + (100 - self.min_duty) * magnitude / 100.0
        backwards = speed < 0
        self._dir.value(1 if backwards != self.invert else 0)
        self._pwm.duty_u16(int(duty * 65535 / 100))

    def stop(self):
        self.speed = 0
        self._pwm.duty_u16(0)


class DriveBase:
    """Left and right drive motors."""

    def __init__(self, left, right):
        self.left = left
        self.right = right

    @classmethod
    def from_config(cls):
        import config
        left = Motor(config.MOTOR_LEFT_DIR_PIN, config.MOTOR_LEFT_PWM_PIN, config.MOTOR_LEFT_INVERT,
                     config.MOTOR_PWM_FREQ_HZ, config.MOTOR_MIN_DUTY)
        right = Motor(config.MOTOR_RIGHT_DIR_PIN, config.MOTOR_RIGHT_PWM_PIN, config.MOTOR_RIGHT_INVERT,
                      config.MOTOR_PWM_FREQ_HZ, config.MOTOR_MIN_DUTY)
        return cls(left, right)

    def drive(self, left_speed, right_speed):
        self.left.set(left_speed)
        self.right.set(right_speed)

    def stop(self):
        self.left.stop()
        self.right.stop()
