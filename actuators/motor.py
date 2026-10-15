"""
Drive motors on the department motor driver (one DIR pin + one PWM pin each).

The driver turns the motor one way with DIR = 0 and the other way with
DIR = 1; the PWM duty cycle sets the speed. Whether DIR = 0 is "forward" for a
wheel depends on how its motor is mounted and wired, so each motor has an
`invert` flag. Find the right flags with tests/test_motors.py.

Speeds are percent of full power, -100 .. +100 (negative = backwards).
`min_duty` lifts every non-zero speed above the motor's dead band: with
min_duty = 20, a request of 1 % gives 20.8 % and 100 % stays 100 %.

Usage
    from actuators.motor import DriveBase
    drive = DriveBase.from_config()
    drive.drive(40, 40)      # straight on
    drive.drive(-30, 30)     # spin left on the spot
    drive.stop()
"""

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

    def duty_percent(self):
        """The PWM duty cycle actually applied, in percent."""
        return self._pwm.duty_u16() * 100.0 / 65535


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
