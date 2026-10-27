
"""
The drop-off at a rack slot: let go of the box, then push it in.

ADDED (2026-10-27). The robot has driven into the drop slot and stopped,
holding the box (servo2 closed). release_and_push() then (settings in
config.py, see the comment block above DROP_MOVE_SPEED):
    1. let go: servo2 opens by GRIPPER_RELEASE_DEG (release_box)
    2. back off straight for DROP_BACK_MS
    3. close the gripper to SERVO_PUSH_ANGLE (below its normal safe range -
       allowed for this push only, the user's decision)
    4. drive forward for DROP_PUSH_MS: the closed gripper pushes the box in
    5. back out for DROP_EXIT_MS
    6. open the gripper again to its carry pose (servo1 up, servo2 open),
       so the next pick-up starts with an open gripper
It WAITS (blocks) until everything is done, a few seconds; the robot stands
still at the end. Every step prints one line starting with "dropoff: " (the
simulator check, sim/check.py "drop-off", reads these lines: keep them).
Because DROP_BACK_MS + DROP_EXIT_MS is longer than DROP_PUSH_MS, the robot
ends a few cm behind the place where it started.

CHANGED (2026-10-27), safety:
  * if a gripper servo has no pulses yet (gripper_start was never run) the
    robot does not move at all and release_and_push() returns False;
  * the push angle is never more than PUSH_MAX_BELOW_SAFE_DEG below servo2's
    safe range, whatever config.SERVO_PUSH_ANGLE says (typo protection);
  * the wheels are stopped even when a move is broken off with Ctrl-C.

ADDED (2026-10-27): drive_in_and_drop() is the WHOLE drop-off, starting
right after the turn from the main line into the drop branch: arm
(servo1) to config.DROP_SERVO1_ANGLE, drive in for config.DROP_DRIVE_MS
following the branch's line, then release_and_push(). Both values come from
the sensor viewer's drop-off calibration. functions.py's "drop_off" mode
and tests/test_drop_run.py call it.

Usage
    from control.dropoff import release_and_push
    release_and_push(robot, motor3, motor4, servo1, servo2)   # True = done
    from control.dropoff import drive_in_and_drop
    drive_in_and_drop(robot, motor3, motor4, servo1, servo2)  # from the turn: drive in, then the above
"""

import utime

import config
from actuators.gripper import gripper_end, release_box
import constants                                  # ADDED (2026-10-27): the partner's DRIVE_SPEED / TURN_SPEED
from sensors.line_processing import INNER_LEFT, INNER_RIGHT, bits_to_str   # ADDED (2026-10-27): line sensor bits

# ADDED (2026-10-27): the push may close servo2 at most this many degrees
# below the low end of its safe range. The user allowed 130 - 10 = 120
# (config.SERVO_PUSH_ANGLE); this stops a typo there (e.g. 12 instead of
# 120) from driving the gripper far into its end stop.
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
        # FIX (2026-10-27): stop the wheels even if the wait is broken off
        # (Ctrl-C = KeyboardInterrupt). Before, they kept turning until the
        # caller's own `finally` (if it had one) switched them off.
        robot.stop(motor3, motor4)


def _push_angle(servo2):
    """ADDED (2026-10-27): the angle servo2 closes to for the push:
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
    """Let go of the box and push it into the slot (steps 1 - 6 above).
    Returns True when done. CHANGED (2026-10-27): returns False, without
    moving anything, if a gripper servo has no pulses yet."""
    robot.stop(motor3, motor4)

    # ADDED (2026-10-27): a servo with no pulses (never reset with
    # gripper_start) ignores every move. The gripper would then be limp
    # while the robot drives back and forth: do nothing instead.
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
    #    CHANGED (2026-10-27): servo1 (arm) first, then servo2, the same
    #    order as actuators/gripper.py gripper_carry (arm up before the
    #    gripper opens). servo1 is normally already up, so it does not move.
    #    servo2 starts below its normal range (at the push angle), so it
    #    also gets low=push_angle: it then opens one degree at a time
    #    instead of first jumping up to the low end of its range.
    servo1.move_slowly(gripper_end(servo1, config.GRIPPER_CARRY.get("servo1", "high")),
                       config.GRIPPER_STEP_MS)
    servo2.move_slowly(gripper_end(servo2, config.GRIPPER_CARRY.get("servo2", "high")),
                       config.GRIPPER_STEP_MS, low=push_angle)
    utime.sleep_ms(config.GRIPPER_PAUSE_MS)
    print("dropoff: done (gripper open again: servo1 %s, servo2 %s deg)" % (servo1.angle(), servo2.angle()))
    return True


def _follow_line_for(robot, motor3, motor4, line, ms):
    """ADDED (2026-10-27): drive forwards for `ms` milliseconds, following
    the line with the SAME rule as the partner's line following
    (functions.py "straight" mode and corrections()):
        both centre sensors white   -> Robot.forward at DRIVE_SPEED (straight)
        only centre-right white     -> Robot.right at TURN_SPEED (left wheel
                                       on, right wheel off: pivot right)
        only centre-left white      -> Robot.left at TURN_SPEED (pivot left)
        both centre sensors black   -> straight on (e.g. past the end of the line)
    then stop. CHANGED (2026-10-27, the user's decision): our own gentle /
    hard steering rules are gone; the partner's rule is the one that works
    on this robot. A status line (time, the four sensors) is printed every
    250 ms."""
    if ms <= 0:
        return
    start = utime.ticks_ms()
    last_print = -1000
    try:
        while True:
            now = utime.ticks_diff(utime.ticks_ms(), start)
            if now >= ms:
                break
            bits = line.read()
            centre_left_white = bool(bits & INNER_LEFT)
            centre_right_white = bool(bits & INNER_RIGHT)
            if centre_right_white and not centre_left_white:
                robot.right(motor3, motor4, constants.TURN_SPEED, constants.TURN_SPEED)
                what = "right"
            elif centre_left_white and not centre_right_white:
                robot.left(motor3, motor4, constants.TURN_SPEED, constants.TURN_SPEED)
                what = "left"
            else:
                robot.forward(motor3, motor4, constants.DRIVE_SPEED, constants.DRIVE_SPEED)
                what = "straight"
            if now - last_print >= 250:
                print("  drive-in %5d ms  line %s  %s" % (now, bits_to_str(bits), what))
                last_print = now
            utime.sleep_ms(5)
    finally:
        # stop the wheels even if the drive is broken off (Ctrl-C)
        robot.stop(motor3, motor4)


def drive_in_and_drop(robot, motor3, motor4, servo1, servo2, line=None):
    """ADDED (2026-10-27): the whole drop-off, starting right after the turn
    from the main line into the drop branch (robot on the branch's line,
    facing the rack, holding the box):
        1. arm (servo1) to config.DROP_SERVO1_ANGLE, one degree at a time
           (Servo.move_slowly keeps it inside servo1's safe range),
        2. drive in for config.DROP_DRIVE_MS (the time measured in the
           calibration window): following the branch's line with the
           partner's rule if `line` (the LineSensorArray) is given, else
           dead straight at DROP_MOVE_SPEED,
        3. let go and push the box in: release_and_push() (steps 1 - 6 at
           the top of this file: let go, back off, close, push, back out,
           open again).
    CHANGED (2026-10-27, the user's order): the arm goes to its angle BEFORE
    the drive, not after it; and the drive follows the branch's line (a
    dead straight drive did not keep the robot on it).
    Returns True when done. Returns False, without moving anything, if a
    gripper servo has no pulses yet (the same check as release_and_push)."""
    robot.stop(motor3, motor4)
    if servo1.angle() is None or servo2.angle() is None:
        print("dropoff: NOT done - the gripper servos have no pulses yet (run gripper_start first)")
        return False

    # 1. arm to the angle it had in the calibration
    servo1.move_slowly(config.DROP_SERVO1_ANGLE, config.GRIPPER_STEP_MS)
    utime.sleep_ms(config.GRIPPER_PAUSE_MS)
    print("dropoff: arm at %s deg" % servo1.angle())

    # 2. into the branch for the time measured with the calibration window,
    #    following its line when the line sensors are given
    if line is not None:
        _follow_line_for(robot, motor3, motor4, line, config.DROP_DRIVE_MS)
        print("dropoff: drove into the branch %d ms (following the line)" % config.DROP_DRIVE_MS)
    else:
        _drive_for(robot, motor3, motor4, config.DROP_DRIVE_MS, reverse=False)
        print("dropoff: drove into the branch %d ms (straight)" % config.DROP_DRIVE_MS)

    # 3. let go, back off, close the gripper, push the box in, back out, open
    return release_and_push(robot, motor3, motor4, servo1, servo2)
