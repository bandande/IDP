"""
Gripper servos: move each servo to 0, 135 and 270 degrees, 2 s apart.
(This is the test that used to run inside the old servos.py.)

    from tests.test_servos import test_servos; test_servos()
Stop with Ctrl-C (the servos then go limp).
"""

from utime import sleep

import config
from actuators.servo import Servo


def test_servos(rounds=None, pause_s=2):
    servos = {name: Servo(pin) for name, pin in config.SERVO_PINS.items()}
    done = 0
    try:
        while rounds is None or done < rounds:
            for angle in (0, 135, 270):
                print("all servos -> %d degrees" % angle)
                for servo in servos.values():
                    servo.set_angle(angle)
                sleep(pause_s)
            done += 1
    finally:
        for servo in servos.values():
            servo.off()


if __name__ == "__main__":
    test_servos()
