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
                        continue           # stays where its reset ended
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
