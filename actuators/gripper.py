import utime

import config


def gripper_end(servo, end):
    """The angle of the "low" or "high" end of a servo's safe range."""
    if end == "low":
        return servo.low
    elif end == "medium":
        return servo.medium
    else:
        return servo.high


def gripper_lift(servo1, servo2):
    """Gripper to its lift pose (config.GRIPPER_CARRY: servo1 up, servo2
    open), servo1 first so the arm is up before the gripper opens."""
    for name, servo in (("servo1", servo1), ("servo2", servo2)):
        servo.move_slowly(gripper_end(servo, config.GRIPPER_LIFT.get(name, "high")), config.GRIPPER_STEP_MS)


def gripper_carry(servo1, servo2):
    """Gripper to its carry pose (config.GRIPPER_CARRY: servo1 up, servo2
    open), servo1 first so the arm is up before the gripper opens."""
    for name, servo in (("servo1", servo1), ("servo2", servo2)):
        servo.move_slowly(gripper_end(servo, config.GRIPPER_CARRY.get(name, "medium")), config.GRIPPER_STEP_MS)
    print(servo1.angle)

def gripper_safe(servo1, servo2):
    """Gripper to its carry pose (config.GRIPPER_CARRY: servo1 up, servo2
    open), servo1 first so the arm is up before the gripper opens."""
    for name, servo in (("servo1", servo1), ("servo2", servo2)):
        print(config.GRIPPER_SAFE.get(name, "low"))
        servo.move_slowly(gripper_end(servo, config.GRIPPER_SAFE.get(name, "low")), config.GRIPPER_STEP_MS)

def gripper_start(servo1, servo2):
    """First gripper movement after the start press: safe reset of both
    servos (their position is unknown), then the carry pose."""
    for name, servo in (("servo1", servo1), ("servo2", servo2)):
        carry = gripper_end(servo, config.GRIPPER_CARRY.get(name, "high"))
        servo.reset(config.SERVO_RESET_SETTLE_MS, config.SERVO_RESET_STEP_MS, target=carry)
    utime.sleep_ms(config.GRIPPER_PAUSE_MS)
    gripper_safe(servo1, servo2)       # no move now; still needed if a servo has a reset path
    print("gripper ready: servo1 %s deg, servo2 %s deg" % (servo1.angle(), servo2.angle()))


def grab_box(servo1, servo2):
    """Pick the box up, standing in front of it: servo1 down to its low end,
    servo2 down to its low end (gripper closes), servo1 up to its high end."""
    for servo, end, what in (
            (servo1, "low", "arm down"), 
            (servo2, "low", "gripper closed"),
        ):
        servo.move_slowly(gripper_end(servo, end), config.GRIPPER_STEP_MS)
        print("gripper: %s (%s deg)" % (what, servo.angle()))
        utime.sleep_ms(config.GRIPPER_PAUSE_MS)

def grab_finish_box(servo1, servo2):
    """Pick the box up, standing in front of it: servo1 down to its low end,
    servo2 down to its low end (gripper closes), servo1 up to its high end."""
    for servo, end, what in [(servo1, "medium", "arm up")]:
        servo.move_slowly(gripper_end(servo, end), config.GRIPPER_STEP_MS)
        print("gripper: %s (%s deg)" % (what, servo.angle()))
        utime.sleep_ms(config.GRIPPER_PAUSE_MS)


def release_box(servo2):
    """Let go of the box: servo2 opens by config.GRIPPER_RELEASE_DEG."""
    if servo2.angle() is None:
        return
    servo2.move_slowly(servo2.angle() + config.GRIPPER_RELEASE_DEG, config.GRIPPER_STEP_MS)
    print("gripper: released (servo2 %s deg)" % servo2.angle())
    utime.sleep_ms(config.GRIPPER_PAUSE_MS)
