"""
Pick-up calibration test: run ONLY the pick-up sequence (control/pickup.py),
starting with the robot already standing in a red / green box stub.
The robot does NOT drive the route from the start box.

ADDED (2026-10-27)

How to run (on the robot, from the REPL)
    from tests.test_pickup import test_pickup; test_pickup()
    test_pickup(seconds=30)      # shorter overall safety limit (default 60 s)
    test_pickup(box="RED_2")     # ADDED (2026-10-27): the box the robot stands in
                                 # (GREEN_1 default, GREEN_2, RED_1, RED_2): it sets
                                 # the U-turn direction, so give the right one

Before the start press
  1. Put the robot by hand where it would be just after turning from the
     main line into the box stub: the WHEELS on the junction, the LINE
     SENSORS on the box's line, the robot FACING the box.
  2. Start the test. It sets up the hardware like main.py does and prints
     the magnet sensor's zero point (keep other magnets away from the robot
     while it measures; the box itself is far enough away).
  3. Press the start/stop button (config.START_STOP_SENSOR) once. The test
     waits config.START_DELAY_MS so your hand is clear, resets the gripper
     the safe way (gripper_start), then runs the pick-up sequence:
     follow the line to the cross line, drive up to the box, grab it,
     turn round and drive back to the main line.

While it runs
  * A line is printed whenever the sequence moves on to a new phase, and a
    status line every PRINT_EVERY_MS: time since the start press, phase,
    the four line sensors ("#" = white, "." = black, left to right) and the
    last front distance the sequence measured.
  * Press the start/stop button again to ABORT: the motors stop at once.
    (While the gripper moves - about 2 s, the wheels stand still then - the
    press is remembered and the test stops right after the grab.)
  * After `seconds` (counted from the start press) the test stops by itself.
  * Ctrl-C also stops the motors.
  * ADDED (2026-10-27): if the GRAB distance sensor did not start, a warning
    is printed (and the red LED is on) before the press, and after the press
    the pick-up fails at once: the robot does not drive at all.

At the end
  * The motors are stopped and one summary line is printed:
        pickup test result: <phase> ok=<True/False> - <message> (<time> ms)
    <time> is how long the pick-up sequence itself ran (from pick.start()).
  * After a successful pick-up the magnet pole of the box is read and printed
    together with the rack level it means (config.MAG_POLE_LEVEL).
  * The servos are NOT switched off: they keep holding the box. Power the
    robot down (or run tests/test_servos.py) to let go of it.
Tune the pick-up with the PICKUP_*, GRAB_* and GRIPPER_* values in config.py.

PC simulator (the start button is pressed automatically at 0.3 s); this
puts the wheels on the green box stub's junction, facing the box:
    python -m sim.run tests.test_pickup:test_pickup --at W_PAD1 --heading east --back 0

Safety
  * The servos only move through actuators/gripper.py and the Servo class
    (actuators/servo.py), which keep every angle inside
    config.SERVO_SAFE_RANGE. This file never writes a servo PWM duty.
  * Whatever happens (end, abort, error, Ctrl-C) both drive motors are
    switched off in a `finally` block, the same way main.py does it.
"""

import machine
import utime

import config
from sensors.line_processing import OUTER_LEFT, INNER_LEFT, INNER_RIGHT, OUTER_RIGHT, bits_to_str


# How often the status line is printed while the sequence runs (ms).
PRINT_EVERY_MS = 250


def _store_line_sensors(robot, bits):
    """Copy one line-sensor reading into the robot, exactly like
    functions.update_sensors() does in main.py: first remember the old
    values as the "previous" ones, then store the new ones
    (1 = the sensor sees white = the line, 0 = black)."""
    robot.left_sensor_previous = robot.left_sensor
    robot.front_left_sensor_previous = robot.front_left_sensor
    robot.front_right_sensor_previous = robot.front_right_sensor
    robot.right_sensor_previous = robot.right_sensor

    robot.left_sensor = 1 if bits & OUTER_LEFT else 0
    robot.front_left_sensor = 1 if bits & INNER_LEFT else 0
    robot.front_right_sensor = 1 if bits & INNER_RIGHT else 0
    robot.right_sensor = 1 if bits & OUTER_RIGHT else 0


def _wait_for_start_press(button):
    """Wait for ONE press of the start/stop button. button.pressed() is True
    once per press, so it is simply asked every 5 ms.
    Returns the utime.ticks_ms() time of the press."""
    print("press the start/stop button (%s) to start the pick-up" % config.START_STOP_SENSOR)
    while not button.pressed():
        utime.sleep_ms(5)
    return utime.ticks_ms()


def test_pickup(seconds=60, box="GREEN_1"):
    """Run the pick-up sequence once from inside a box stub (see the top of
    this file). seconds = overall safety limit, counted from the start press
    (None = no limit). box: ADDED (2026-10-27) which box the robot stands in
    ("GREEN_1", "GREEN_2", "RED_1" or "RED_2"); it only decides the way the
    U-turn spins (config.PICKUP_UTURN_DIRECTION, away from the neighbour box)."""
    # ADDED (2026-10-27): a mistyped box name (e.g. "green_1") used to fall
    # back to a "left" U-turn without a word, which spins TOWARDS the
    # neighbour box for GREEN_1 / RED_2. Refuse it before anything moves.
    if box not in config.PICKUP_UTURN_DIRECTION:
        raise ValueError("box must be one of %s, not %r"
                         % (", ".join(sorted(config.PICKUP_UTURN_DIRECTION)), box))

    # The robot modules are imported here, inside the function, so that
    # importing this file stays cheap (functions.py is large).
    # NOTE: functions must be imported BEFORE motor. motor.py imports
    # functions, and functions.py imports Motor from motor, so importing
    # motor first fails with an ImportError (the same order as main.py).
    import constants
    import functions
    import motor
    from sensors.line_sensor import LineSensorArray
    from actuators.servo import Servo
    from actuators.gripper import gripper_start
    from control.task_sensors import TaskSensors
    from control.start_stop import StartStopButton
    from control.pickup import PickUp

    try:
        # ---- 1. Hardware, set up the same way as
        # functions.initialise_main_variables() (used by main.py) --------------
        # Drive motors: motor3 = left wheel, motor4 = right wheel. Robot's
        # forward() / backward() / ... use these speeds when no speed is given.
        motor3 = motor.Motor(dirPin=4, PWMPin=5)
        motor4 = motor.Motor(dirPin=7, PWMPin=6)
        motor3.forward_speed = constants.DRIVE_SPEED
        motor3.backward_speed = constants.DRIVE_SPEED
        motor4.forward_speed = constants.DRIVE_SPEED
        motor4.backward_speed = constants.DRIVE_SPEED

        # The four line sensors, ordered outer-left, inner-left, inner-right,
        # outer-right (the same pins main.py uses, taken from config.py).
        line = LineSensorArray([config.LINE_OUTER_LEFT_PIN, config.LINE_INNER_LEFT_PIN,
                                config.LINE_INNER_RIGHT_PIN, config.LINE_OUTER_RIGHT_PIN],
                               config.LINE_WHITE_IS_HIGH)

        robot = functions.Robot()

        # Task sensors (magnet, front / grab distance, IR). calibrate()
        # measures the magnet sensor's no-magnet zero point, so it runs now,
        # BEFORE the start press, while no magnet is near the robot.
        tasks = TaskSensors.from_config()
        print("magnet sensor zero point: %s mV" % tasks.calibrate())
        # ADDED (2026-10-27): say it loudly before anything moves if the grab
        # sensor is missing - the pick-up then fails at once (CHANGED: it
        # used to drive to the cross line first and stop at "to_box").
        if tasks.grab_tof is None:
            # the pins are looked up in config, so this stays right after re-wiring
            specs = [t for t in config.TOF_SENSORS if t["name"] == config.GRAB_TOF]
            if specs:
                pins = "physical pins %s / %s" % (config.physical_pin(specs[0]["sda"]),
                                                  config.physical_pin(specs[0]["scl"]))
            else:
                # FIX (2026-10-27): this used to crash with an IndexError
                pins = "NOT in config.TOF_SENSORS"
            print("WARNING: the GRAB distance sensor (%s, front, %s) did not start:"
                  " the pick-up will fail at once, the robot will not move. Check its wiring first."
                  % (config.GRAB_TOF, pins))
        else:
            print("grab distance sensor %s ready" % config.GRAB_TOF)

        # Gripper servos, taken BY NAME: MicroPython dicts do not keep their
        # order. Creating them does not move them (no pulses yet).
        servo1 = Servo(config.SERVO_PINS["servo1"])
        servo2 = Servo(config.SERVO_PINS["servo2"])

        button = StartStopButton.from_config()

        # The pick-up sequence. Made now, so a setup error shows up before
        # anything moves; it only starts driving after pick.start().
        uturn = config.PICKUP_UTURN_DIRECTION[box]     # ADDED (2026-10-27); box checked above
        pick = PickUp(robot, motor3, motor4, servo1, servo2, tasks, uturn)

        # ---- 2. Instructions, then wait for the start press -----------------
        print("pick-up test in box %s, U-turn spins %s (overall limit %s s)" % (box, uturn, seconds))
        print("place the robot: wheels on the junction, line sensors on the box's line, facing the box")
        print("press the button again during the run to abort")
        # ADDED (2026-10-27): a light you can see on the robot: GREEN on =
        # ready, waiting for the press (red too if the grab sensor is missing).
        # A missing / broken LED board must not stop the test, hence the try.
        leds = None
        try:
            from actuators.leds import StatusLeds
            leds = StatusLeds.from_config()
            leds.show(green=True, red=tasks.grab_tof is None)
        except Exception as e:
            print("LEDs not available (%r)" % (e,))
        t0 = _wait_for_start_press(button)
        if leds is not None:
            leds.off()                  # pressed: the light goes out, the run starts
        # Code that measures time from robot.start (as main.py does) then
        # counts from the start press too, like now_ms below.
        robot.start = t0
        print("starting in %d ms" % config.START_DELAY_MS)
        utime.sleep_ms(config.START_DELAY_MS)

        # ---- 3. Gripper: safe reset of both servos, then the carry pose -----
        gripper_start(servo1, servo2)

        # ---- 4. Run the pick-up sequence -------------------------------------
        # now_ms = milliseconds since the start press.
        now_ms = utime.ticks_diff(utime.ticks_ms(), t0)
        pick_start_ms = now_ms
        pick.start(now_ms)
        print(">>> pick-up started at %d ms, phase %s" % (now_ms, pick.phase))

        finished = False        # True once pick.update() reports the end
        stop_reason = None      # why the test stopped early (abort / time limit)
        last_phase = pick.phase
        last_print_ms = now_ms
        while True:
            now_ms = utime.ticks_diff(utime.ticks_ms(), t0)

            # A new press of the start/stop button = abort.
            if button.pressed():
                robot.stop(motor3, motor4)
                print("pickup test aborted by the button")
                stop_reason = "aborted by the button"
                break

            # Overall safety time limit.
            if seconds is not None and now_ms >= seconds * 1000:
                robot.stop(motor3, motor4)
                print("pickup test timed out: not finished after %s s" % seconds)
                stop_reason = "timed out after %s s" % seconds
                break

            # Read the four line sensors ONCE per loop and store them in the
            # robot, then let the pick-up sequence take its next step.
            bits = line.read()
            _store_line_sensors(robot, bits)
            finished = pick.update(now_ms)

            # Report every change of phase, and a status line now and then.
            if pick.phase != last_phase:
                print(">>> phase %s -> %s at %d ms (line %s, distance %s mm)"
                      % (last_phase, pick.phase, now_ms, bits_to_str(bits), pick.last_distance_mm))
                last_phase = pick.phase
            if now_ms - last_print_ms >= PRINT_EVERY_MS:
                print("  %6d ms  %-14s line %s  distance %s mm"
                      % (now_ms, pick.phase, bits_to_str(bits), pick.last_distance_mm))
                last_print_ms = now_ms

            if finished:
                break
            utime.sleep_ms(5)

        # ---- 5. End: stop the motors and print the result --------------------
        robot.stop(motor3, motor4)
        # Read the clock again: the last pick.update() may have waited a
        # while (the grab blocks while the servos move), so now_ms can be old.
        end_ms = utime.ticks_diff(utime.ticks_ms(), t0)
        run_ms = end_ms - pick_start_ms
        if finished:
            ok = pick.ok
            message = pick.message
        else:
            ok = False
            message = stop_reason
        print("pickup test result: %s ok=%s - %s (%d ms)" % (pick.phase, ok, message, run_ms))

        # After a successful pick-up the box (and its magnet) is in the
        # gripper: read which pole it shows. The robot is standing still.
        if finished and pick.ok:
            # CHANGED (2026-10-27): the pole is read by the sequence itself,
            # right after the grab (pick.pole); this only repeats the result.
            print("magnet pole of the box: %s -> rack level %s (only reported, no turn yet)"
                  % (pick.pole.upper() if pick.pole else "NOT FOUND", pick.level))
    finally:
        # Safety: switch both drive motors off whatever happened (end, abort,
        # error or Ctrl-C), the same way main.py's finally does (PWM pins 5
        # and 6). The servos are NOT touched: they keep holding the box.
        for pwm_pin in (5, 6):
            machine.PWM(machine.Pin(pwm_pin)).duty_u16(0)
        print("drive motors off")


if __name__ == "__main__":
    test_pickup()
