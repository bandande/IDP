"""
The gripper: the five helper functions that move the two gripper servos.

ADDED (2026-10-27): moved here unchanged from functions.py, so that other
modules (control/pickup.py, main.py, functions.py) can use the gripper
without importing the whole of functions.py. The code and its behaviour are
exactly the same as before.

The servos only ever move through actuators/servo.py `Servo` methods
(reset(), move_slowly()), which clamp every angle into
config.SERVO_SAFE_RANGE. Nothing here writes a PWM duty directly.

Which end of a servo's safe range does what (see config.py):
    servo1 low  = arm down,         servo1 high = arm up
    servo2 low  = gripper closed,   servo2 high = gripper open

Usage
    from actuators.gripper import gripper_start, grab_box, release_box
    gripper_start(servo1, servo2)   # once, after the start press
    grab_box(servo1, servo2)        # standing in front of a box
    release_box(servo2)             # at the drop-off
"""

import utime

import config


# ADDED (2026-10-26, test build): the gripper, from our branch (xz-dev).
# Both servos only ever move inside config.SERVO_SAFE_RANGE, one degree at a
# time (Servo.move_slowly), and a servo with no pulses is only started with
# its safe reset (straight to the middle of its range).
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
    # CHANGED (2026-10-27): reset each servo STRAIGHT to its carry pose
    # (servo1 up, servo2 open - inside the safe range), servo1 first so the
    # arm is up before the gripper opens. Before, both went to the middle of
    # their range first and then to the carry pose: two moves, the arm
    # dipping down and lifting again at every start.
    for name, servo in (("servo1", servo1), ("servo2", servo2)):
        carry = gripper_end(servo, config.GRIPPER_CARRY.get(name, "high"))
        servo.reset(config.SERVO_RESET_SETTLE_MS, config.SERVO_RESET_STEP_MS, target=carry)
    utime.sleep_ms(config.GRIPPER_PAUSE_MS)
    gripper_safe(servo1, servo2)       # no move now; still needed if a servo has a reset path
    print("gripper ready: servo1 %s deg, servo2 %s deg" % (servo1.angle(), servo2.angle()))


def grab_box(servo1, servo2):
    """Pick the box up, standing in front of it: servo1 down to its low end,
    servo2 down to its low end (gripper closes), servo1 up to its high end."""
    for servo, end, what in ((servo1, "low", "arm down"), (servo2, "low", "gripper closed"),
                             (servo1, "medium", "arm up")):
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
