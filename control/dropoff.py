import utime

import config
from actuators.gripper import gripper_end, release_box

PUSH_MAX_BELOW_SAFE_DEG = 10


def _drive_for(robot, motor3, motor4, ms, reverse):
    """Drive straight (both wheels DROP_MOVE_SPEED) forwards or backwards
    for `ms` milliseconds, then stop. Robot.forward() must never get a
    speed of 0 (it would use full speed), hence the max(1, ...)."""
    if ms <= 0:
        return
    speed = max(1, config.DROP_MOVE_SPEED)
    robot.forward(motor3, motor4, speed, speed, reverse=reverse)
    try:
        utime.sleep_ms(ms)
    finally:
        robot.stop(motor3, motor4)


def _push_angle(servo2):
    """The angle servo2 closes to for the push:
    config.SERVO_PUSH_ANGLE["servo2"] (or the low end of its safe range if
    it is not set), but never more than PUSH_MAX_BELOW_SAFE_DEG below that
    low end."""
    angle = config.SERVO_PUSH_ANGLE.get("servo2", servo2.low)
    lowest = servo2.low - PUSH_MAX_BELOW_SAFE_DEG
    if angle < lowest:
        print("dropoff: WARNING SERVO_PUSH_ANGLE %s is more than %d deg below servo2's safe range - using %d"
              % (angle, PUSH_MAX_BELOW_SAFE_DEG, lowest))
        angle = lowest
    return angle


def release_and_push(robot, motor3, motor4, servo1, servo2):
    """Let go of the box and push it into the slot.
    Returns True when done, or False, without
    moving anything, if a gripper servo has no pulses yet."""
    robot.stop(motor3, motor4)

    if servo1.angle() is None or servo2.angle() is None:
        print("dropoff: NOT done - the gripper servos have no pulses yet (run gripper_start first)")
        return False

    push_angle = _push_angle(servo2)

    # 1. let go of the box
    release_box(servo2)
    print("dropoff: released")

    # 2. back off a little, so the gripper can close in front of the box
    _drive_for(robot, motor3, motor4, config.DROP_BACK_MS, reverse=True)
    print("dropoff: backed off %d ms" % config.DROP_BACK_MS)

    # 3. close the gripper to the push angle. low=push_angle lets THIS move
    #    go below servo2's normal safe range (only here, see config.py).
    servo2.move_slowly(push_angle, config.GRIPPER_STEP_MS, low=push_angle)
    utime.sleep_ms(config.GRIPPER_PAUSE_MS)
    print("dropoff: gripper closed for the push (servo2 %s deg)" % servo2.angle())

    # 4. push the box in with the closed gripper
    _drive_for(robot, motor3, motor4, config.DROP_PUSH_MS, reverse=False)
    print("dropoff: pushed %d ms" % config.DROP_PUSH_MS)

    # 5. back out, clear of the box
    _drive_for(robot, motor3, motor4, config.DROP_EXIT_MS, reverse=True)
    print("dropoff: backed out %d ms" % config.DROP_EXIT_MS)

    # 6. gripper back to the carry pose.
    servo1.move_slowly(gripper_end(servo1, config.GRIPPER_CARRY.get("servo1", "high")),
                       config.GRIPPER_STEP_MS)
    servo2.move_slowly(gripper_end(servo2, config.GRIPPER_CARRY.get("servo2", "high")),
                       config.GRIPPER_STEP_MS, low=push_angle)
    utime.sleep_ms(config.GRIPPER_PAUSE_MS)
    print("dropoff: done (gripper open again: servo1 %s, servo2 %s deg)" % (servo1.angle(), servo2.angle()))
    return True


