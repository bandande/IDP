"""
Drop-off calibration test: run ONLY the drop-off at a rack slot
(control/dropoff.py release_and_push), with the robot already standing where
it stops in a drop slot. The robot does NOT drive any route.

ADDED (2026-10-27)

How to run (on the robot, from the REPL)
    from tests.test_dropoff import test_dropoff; test_dropoff()

Three presses of the start/stop button (config.START_STOP_SENSOR):
  1st press: the gripper is reset the safe way and opens (carry pose:
             servo1 up, servo2 open). Put the box between the open jaws.
  2nd press: the gripper closes on the box (servo2 to the low end of its
             safe range) and holds it. Now put the robot by hand where it
             stops in a drop slot (as at the end of the route), facing the
             rack.
  3rd press: the drop-off runs: let go, back off, close the gripper to
             config.SERVO_PUSH_ANGLE, push the box in, back out, open the
             gripper again. Then the test ends.
The GREEN LED is on whenever the test waits for a press. Presses made while
the robot is busy (gripper moving, start delay) are ignored (ADDED
2026-10-27), so a double press cannot skip a step - e.g. close the gripper
while your fingers are still putting the box in.

Tune with DROP_MOVE_SPEED, DROP_BACK_MS, DROP_PUSH_MS, DROP_EXIT_MS,
GRIPPER_RELEASE_DEG and SERVO_PUSH_ANGLE in config.py. Expect the robot to
end a few cm behind where it started (DROP_BACK_MS + DROP_EXIT_MS is longer
than DROP_PUSH_MS).

PC simulator (presses at 0.3 s, 4 s and 8 s), robot at the end of the rung to
LEFT_BOTTOM_DROP_1 (ORANGE_W1), facing the rack (east); the rear axle is
0.20 m behind the rung end, so the line sensors are on it:
    python -m sim.run tests.test_dropoff:test_dropoff --at ORANGE_W1 --heading east --press 0.3 4 8 --time 20
(FIX 2026-10-27: this said "--at W1 --heading west", which puts the robot at
the other end of the rung, facing AWAY from the rack.) The same run is the
"drop-off" scenario of python -m sim.check.

Safety
  * The servos only move through actuators/gripper.py, control/dropoff.py
    and the Servo class, which keep every angle inside config.SERVO_SAFE_RANGE
    (the push angle is the one allowed exception, see config.py).
  * Whatever happens (end, error, Ctrl-C) both drive motors are switched off
    in a `finally` block, the same way main.py does it. The drop-off itself
    cannot be aborted half-way with the button (it is short, a few seconds);
    Ctrl-C stops it.
"""

import machine
import utime

import config


def _wait_for_press(button, leds, what):
    """Print `what`, light the GREEN LED and wait for ONE press of the
    start/stop button (button.pressed() is True once per press)."""
    # ADDED (2026-10-27): forget presses made while the robot was busy
    # (the crash sensor remembers short presses even while nobody asks).
    # Otherwise an extra press during the gripper reset would make this
    # wait end at once and skip a step.
    while button.pressed():
        pass
    print("press the start/stop button to %s" % what)
    if leds is not None:
        leds.show(green=True)
    while not button.pressed():
        utime.sleep_ms(5)
    if leds is not None:
        leds.off()


def test_dropoff():
    """Run the drop-off once (see the top of this file)."""
    # Imported here so that importing this file stays cheap. functions must
    # come before motor (motor.py imports functions), as in main.py.
    import constants
    import functions
    import motor
    from actuators.servo import Servo
    from actuators.gripper import gripper_start, gripper_end
    from control.start_stop import StartStopButton
    from control.dropoff import release_and_push

    try:
        # ---- 1. Hardware, set up like functions.initialise_main_variables() --
        # motor3 = left wheel, motor4 = right wheel.
        motor3 = motor.Motor(dirPin=4, PWMPin=5)
        motor4 = motor.Motor(dirPin=7, PWMPin=6)
        motor3.forward_speed = motor3.backward_speed = constants.DRIVE_SPEED
        motor4.forward_speed = motor4.backward_speed = constants.DRIVE_SPEED
        robot = functions.Robot()
        # Servos taken BY NAME (MicroPython dicts do not keep their order).
        # Creating them does not move them (no pulses yet).
        servo1 = Servo(config.SERVO_PINS["servo1"])
        servo2 = Servo(config.SERVO_PINS["servo2"])
        button = StartStopButton.from_config()
        # LEDs are only a convenience: a missing LED board must not stop the test.
        leds = None
        try:
            from actuators.leds import StatusLeds
            leds = StatusLeds.from_config()
        except Exception as e:
            print("LEDs not available (%r)" % (e,))

        print("drop-off test: back %d ms, push %d ms, exit %d ms at %d %%, push angle %s"
              % (config.DROP_BACK_MS, config.DROP_PUSH_MS, config.DROP_EXIT_MS,
                 config.DROP_MOVE_SPEED, config.SERVO_PUSH_ANGLE.get("servo2")))

        # ---- 2. 1st press: reset the gripper and open it --------------------
        _wait_for_press(button, leds, "open the gripper (then put the box between its jaws)")
        utime.sleep_ms(config.START_DELAY_MS)          # hand clear of the gripper
        gripper_start(servo1, servo2)

        # ---- 3. 2nd press: close on the box and hold it -----------------------
        _wait_for_press(button, leds, "close the gripper on the box")
        servo2.move_slowly(gripper_end(servo2, "low"), config.GRIPPER_STEP_MS)
        print("holding the box (servo1 %s, servo2 %s deg)" % (servo1.angle(), servo2.angle()))

        # ---- 4. 3rd press: the drop-off ---------------------------------------
        print("put the robot where it stops in a drop slot, facing the rack")
        _wait_for_press(button, leds, "start the drop-off")
        utime.sleep_ms(config.START_DELAY_MS)          # hand clear of the robot
        t0 = utime.ticks_ms()
        done = release_and_push(robot, motor3, motor4, servo1, servo2)
        robot.stop(motor3, motor4)
        # CHANGED (2026-10-27): release_and_push() says whether it ran
        print("drop-off test result: %s in %d ms (servo1 %s, servo2 %s deg)"
              % ("done" if done else "NOT done", utime.ticks_diff(utime.ticks_ms(), t0),
                 servo1.angle(), servo2.angle()))
    finally:
        # Safety: both drive motors off whatever happened (PWM pins 5 and 6,
        # as in main.py). The servos keep their position.
        for pwm_pin in (5, 6):
            machine.PWM(machine.Pin(pwm_pin)).duty_u16(0)
        print("drive motors off")


if __name__ == "__main__":
    test_dropoff()
