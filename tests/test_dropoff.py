import machine
import utime

import config


def _wait_for_press(button, leds, what):
    """Print `what`, light the GREEN LED and wait for ONE press of the
    start/stop button (button.pressed() is True once per press)."""
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
    """Run the drop-off once."""
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
