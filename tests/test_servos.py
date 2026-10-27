"""
Gripper servos: reset each servo the safe way, then move servo1 inside its
SAFE range (config.SERVO_SAFE_RANGE), 2 s apart.
(This is the test that used to run inside the old servos.py.)

    from tests.test_servos import test_servos; test_servos()
Stop with Ctrl-C (the servos then go limp).

CHANGED (2026-10-23): it used to move every servo to 0, 135 and 270 degrees.
The gripper cannot reach the whole range: driving it into its end stop
stalls the servo, and the current can reset the Pico. Now:
  * every servo starts with reset() (config.SERVO_RESET, or straight to
    the middle of its safe range), done ONCE;
  * then each round: a servo WITHOUT a reset path goes to the low end, the
    middle and the high end of its safe range (CHANGED 2026-10-26: with the
    new gripper both servos do this); a servo with a reset path stays at
    the end of its reset;
  * a servo whose safe range is still None in config.py is left out.
"""

from utime import sleep

import config
from actuators.servo import Servo


def test_servos(rounds=None, pause_s=2):
    servos = {}
    for name, pin in config.SERVO_PINS.items():
        if config.SERVO_SAFE_RANGE.get(name) is None:
            print("%s left out: no safe range in config.SERVO_SAFE_RANGE yet" % name)
            continue
        servos[name] = Servo(pin)          # safe range and reset path come from config.py
    try:
        for name, servo in servos.items():
            angle = servo.reset(config.SERVO_RESET_SETTLE_MS, config.SERVO_RESET_STEP_MS)
            print("%s reset -> %d degrees (safe range %d-%d)" % (name, angle, servo.low, servo.high))
        sleep(pause_s)
        done = 0
        while rounds is None or done < rounds:
            for step in ("low end", "middle", "high end"):
                for name, servo in servos.items():
                    if servo.reset_path is not None:
                        continue           # stays where its reset ended (CHANGED 2026-10-27: no servo has a reset path now)
                    if step == "low end":
                        angle = servo.low
                    elif step == "middle":
                        angle = (servo.low + servo.high) // 2
                    else:
                        angle = servo.high
                    print("%s -> %d degrees (%s)" % (name, angle, step))
                    servo.set_angle(angle)
                sleep(pause_s)
            done += 1
    finally:
        for servo in servos.values():
            servo.off()


if __name__ == "__main__":
    test_servos()
