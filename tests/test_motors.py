"""
Step 9: drive motors - which wheel is which, which way is forward, dead band.

SAFETY: for the first run put the robot on a stand with the wheels off the
table. The motors only move after a press of the start/stop button
(config.START_STOP_SENSOR); press it again at any time to stop everything.
Ctrl-C also stops the motors.

test_motors(): after the press (yellow LED = waiting, green = running):
  1. LEFT wheel forward      -> only the LEFT wheel turns, forwards
  2. RIGHT wheel forward     -> only the RIGHT wheel turns, forwards
  3. both forward            -> the robot would drive straight ahead
  4. spin left               -> left wheel backwards, right wheel forwards
  5. spin right              -> left wheel forwards, right wheel backwards
  If step 1 moved the right wheel: swap the MOTOR_LEFT_* and MOTOR_RIGHT_*
  pins in config.py. If a wheel turned backwards when it should go forwards:
  flip MOTOR_LEFT_INVERT / MOTOR_RIGHT_INVERT. Then run it again.

ramp_test(): both wheels forward from 0 to 60 % in 5 % steps. Note the
first value at which BOTH wheels turn and put it into MOTOR_MIN_DUTY.
"""

from utime import sleep_ms, ticks_ms, ticks_diff

import config
from actuators.motor import DriveBase
from actuators.leds import StatusLeds
from control.start_stop import StartStopButton


def _wait_for_press(button, leds, wait_for_button):
    if not wait_for_button:
        return True
    print("press the %s crash sensor to start (Ctrl-C to quit)" % config.START_STOP_SENSOR)
    leds.show(yellow=True, red=False, green=False)
    while not button.pressed():
        sleep_ms(5)
    leds.show(yellow=False, green=True)
    sleep_ms(500)
    return True


def _hold(drive, button, left, right, ms):
    """Drive for ms milliseconds. Returns False if the button was pressed (abort)."""
    drive.drive(left, right)
    start = ticks_ms()
    while ticks_diff(ticks_ms(), start) < ms:
        if button.pressed():
            drive.stop()
            print("ABORTED by the button")
            return False
        sleep_ms(5)
    drive.stop()
    return True


def test_motors(speed=40, step_ms=1000, pause_ms=600, wait_for_button=True):
    drive = DriveBase.from_config()
    leds = StatusLeds.from_config()
    button = StartStopButton.from_config()
    steps = (
        ("1. LEFT wheel forward", speed, 0),
        ("2. RIGHT wheel forward", 0, speed),
        ("3. both forward (straight ahead)", speed, speed),
        ("4. spin left (left back, right forward)", -speed, speed),
        ("5. spin right (left forward, right back)", speed, -speed),
    )
    try:
        _wait_for_press(button, leds, wait_for_button)
        for name, left, right in steps:
            print("%s   (left %d %%, right %d %%)" % (name, left, right))
            if not _hold(drive, button, left, right, step_ms):
                break
            sleep_ms(pause_ms)
        print("done. Wrong wheel in step 1 -> swap LEFT/RIGHT pins; wrong direction -> flip *_INVERT")
    finally:
        drive.stop()
        leds.off()
        button.deinit()


def ramp_test(top=60, step=5, step_ms=700, wait_for_button=True):
    drive = DriveBase.from_config()
    leds = StatusLeds.from_config()
    button = StartStopButton.from_config()
    try:
        _wait_for_press(button, leds, wait_for_button)
        speed = step
        while speed <= top:
            duty = config.MOTOR_MIN_DUTY + (100 - config.MOTOR_MIN_DUTY) * speed / 100.0
            print("both wheels forward: speed %d %% -> PWM duty %.0f %%" % (speed, duty))
            if not _hold(drive, button, speed, speed, step_ms):
                break
            speed += step
        print("put the first speed at which both wheels turned into MOTOR_MIN_DUTY")
    finally:
        drive.stop()
        leds.off()
        button.deinit()


if __name__ == "__main__":
    test_motors()
