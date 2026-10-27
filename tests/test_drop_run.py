"""
Drop-off run test: from the main line, turn into a drop branch and do the
whole automatic drop-off, driving with the PARTNER's line following and
navigation (functions.update_sensors + functions.robot_mode_check, exactly
what main.py runs).

ADDED (2026-10-27). CHANGED (2026-10-27, the user's decision): our own line
following and turn for this test are gone again; the partner's code drives.

How to run (on the robot, from the REPL)
    from tests.test_drop_run import test_drop_run; test_drop_run()
    test_drop_run(turn="left")     # the branch is on the left instead
    test_drop_run(seconds=30)      # shorter overall safety limit (default 40 s)

  * At the start the gripper opens (safe reset straight to the carry pose:
    servo1 up, servo2 open).
  * 1st press of the start/stop button: the gripper picks the box up with
    the fixed grab angles (actuators/gripper.py grab_box: servo1 down to
    155, servo2 closed to 130, servo1 up to 173). Put the box in front of
    the gripper first.
  * 2nd press: the run, with the queue main.py would build (used from the
    END):
      1. "straight": the partner's line following up to the next junction,
      2. ["turning", turn, 90]: the partner's turn into the branch,
      3. "drop_off": control/dropoff.py drive_in_and_drop: servo1 to
         config.DROP_SERVO1_ANGLE, follow the branch's line (the partner's
         rule) for config.DROP_DRIVE_MS, let go, back off, close the gripper
         to config.SERVO_PUSH_ANGLE, push the box in, back out, open again.
    It then stops (config.STOP_AFTER_DROP).
Put the robot on the main line, facing the drop branch's junction, at least
10 cm before it, with NO other junction in between. The GREEN LED is on
whenever the test waits for a press. During the run a press of the button
ABORTS it (the motors stop at once; the drop-off itself, about 12 s, cannot
be broken off by the button, Ctrl-C stops it).

PC simulator (presses at 3 and 7 s), rear axle on the west line south of
the first left drop branch (W1), facing north; that branch is on the right:
    python -m sim.run tests.test_drop_run:test_drop_run --pose 0.30,0.80,90 --press 3 7 --time 45

Safety
  * The servos only move through actuators/gripper.py, control/dropoff.py
    and the Servo class, which keep every angle inside config.SERVO_SAFE_RANGE
    (the push angle is the one allowed exception, see config.py).
  * Whatever happens (end, abort, error, Ctrl-C) both drive motors are
    switched off in a `finally` block, the same way main.py does it. The
    servos keep their position.
"""

import machine
import utime

import config
from tests.test_dropoff import _wait_for_press   # wait for ONE press, LED on, stale presses ignored


# How often the status line is printed while the robot drives (ms).
PRINT_EVERY_MS = 500

# No junction is accepted this soon after the start press: it is put into
# the partner's own hold-off (holdoff_until, variables[5]), the same thing
# the partner's code sets after each junction. On the robot a junction was
# once "seen" 0.15 s after the start.
START_HOLDOFF_MS = 300

# The drop slot this test pretends to go to. Only used for the map entry that
# the "drop_off" mode marks as FILLED (in RAM, gone after the test).
TEST_DROP_SLOT = "LEFT_BOTTOM_DROP_1"


def test_drop_run(seconds=40, turn="right"):
    """Run straight -> turn -> drop-off once (see the top of this file).
    seconds = overall safety limit, counted from the 2nd press.
    turn = "right" or "left": the side of the drop branch."""
    if turn not in ("right", "left"):
        raise ValueError("turn must be 'right' or 'left', not %r" % (turn,))

    # Imported here so that importing this file stays cheap (functions.py is
    # large). functions sets up the hardware exactly like main.py.
    import functions as f
    from actuators.gripper import gripper_start, grab_box

    try:
        # ---- Hardware: the same set-up as main.py ----------------------------
        # (line sensors, motors, servos, button, task sensors; it also
        # measures the magnet sensor's zero point.)
        variables = f.initialise_main_variables()
        button = variables[1]
        motor3, motor4 = variables[9], variables[10]
        robot = variables[16]
        servo1, servo2 = variables[19], variables[20]
        # LEDs are only a convenience: a missing LED board must not stop the test.
        leds = None
        try:
            from actuators.leds import StatusLeds
            leds = StatusLeds.from_config()
        except Exception as e:
            print("LEDs not available (%r)" % (e,))

        print("drop run test (partner's line following): next junction -> turn %s, arm to %d deg,"
              " follow the branch line %d ms, release, back off, close, push"
              % (turn, config.DROP_SERVO1_ANGLE, config.DROP_DRIVE_MS))

        # ---- Open the gripper (no press needed) ------------------------------
        gripper_start(servo1, servo2)

        # ---- 1st press: pick the box up with the fixed grab angles -----------
        _wait_for_press(button, leds, "pick the box up (box in front of the gripper)")
        utime.sleep_ms(config.START_DELAY_MS)          # hand clear of the gripper
        grab_box(servo1, servo2)
        print("holding the box (servo1 %s, servo2 %s deg)" % (servo1.angle(), servo2.angle()))

        # ---- The queue main.py would build for this stretch -----------------
        # Used from the END: "straight" first, then the turn, then "drop_off".
        robot.final_objective = TEST_DROP_SLOT
        robot.objective = TEST_DROP_SLOT
        robot.queue = ["drop_off", ["turning", turn, 90], "straight"]

        # ---- 2nd press: run ----------------------------------------------------
        print("put the robot on the main line, facing the drop branch's junction, >= 10 cm before it")
        _wait_for_press(button, leds, "start the run")
        utime.sleep_ms(config.START_DELAY_MS)          # hand clear of the robot
        robot.start = utime.ticks_ms()                  # robot_mode_check counts from here
        # the partner's hold-off: no junction in the first START_HOLDOFF_MS
        variables = variables[:5] + (START_HOLDOFF_MS,) + variables[6:]

        result = None
        last_mode = None
        last_print_ms = 0
        while True:
            now = utime.ticks_diff(utime.ticks_ms(), robot.start)

            # A new press of the start/stop button = abort.
            if button.pressed():
                robot.stop(motor3, motor4)
                result = "aborted by the button (mode %s)" % robot.mode
                break

            # Overall safety time limit.
            if seconds is not None and now >= seconds * 1000:
                robot.stop(motor3, motor4)
                result = "timed out after %s s (mode %s)" % (seconds, robot.mode)
                break

            # One step of the partner's navigation: read the line sensors,
            # then let the mode code act (exactly what main.py's loop does).
            variables = f.update_sensors(variables)
            variables, now = f.robot_mode_check(variables, now)
            robot = variables[16]

            if robot.mode != last_mode:
                print(">>> %6d ms  mode %r %s" % (now, robot.mode, robot.mode_specific))
                last_mode = robot.mode
            if now - last_print_ms >= PRINT_EVERY_MS:
                print("  %6d ms  mode %-10s line %d%d%d%d" % (
                    now, robot.mode, robot.left_sensor, robot.front_left_sensor,
                    robot.front_right_sensor, robot.right_sensor))
                last_print_ms = now

            # The "drop_off" mode clears is_started when STOP_AFTER_DROP is
            # set; without it the queue is simply empty afterwards (the next
            # robot_mode_check would fail on an empty queue: stop here).
            if not variables[6]:
                result = "done (stopped after the drop-off)"
                break
            if robot.mode == "" and not robot.queue:
                result = "done (queue empty)"
                break

        robot.stop(motor3, motor4)
        print("drop run test result: %s (servo1 %s, servo2 %s deg)"
              % (result, servo1.angle(), servo2.angle()))
    finally:
        # Safety: both drive motors off whatever happened (PWM pins 5 and 6,
        # as in main.py). The servos keep their position.
        for pwm_pin in (5, 6):
            machine.PWM(machine.Pin(pwm_pin)).duty_u16(0)
        print("drive motors off")


if __name__ == "__main__":
    test_drop_run()
