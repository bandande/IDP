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

The servo holds the angle it was given (it has its own feedback inside),
but the Pico cannot read the real angle back: angle() only returns the last
angle that was set.

Usage
    from actuators.servo import Servo
    import config
    gripper = Servo(config.SERVO_PINS["servo1"])
    gripper.set_angle(135)
"""

from machine import Pin, PWM


class Servo:
    MIN_DUTY = 1638     # 0.5 ms pulse = 0 degrees
    MAX_DUTY = 8192     # 2.5 ms pulse = 270 degrees
    MAX_ANGLE = 270

    def __init__(self, pin_number):
        self._pwm = PWM(Pin(pin_number))
        self._pwm.freq(50)              # 50 Hz = one pulse every 20 ms
        self._angle = None

    def set_angle(self, angle):
        """Move to `angle` degrees (clamped to 0 .. 270)."""
        if angle < 0:
            angle = 0
        if angle > self.MAX_ANGLE:
            angle = self.MAX_ANGLE
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
