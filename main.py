"""
The robot's main program. MicroPython runs main.py by itself when the Pico
is switched on, so the robot must never move before the button is pressed.

WHAT HAPPENS
  1. Start the hardware: drive motors, line sensors, start/stop button and
     the task sensors (magnet, front distance sensor, IR receiver).
  2. Measure the magnet sensor's zero point. Keep magnets / boxes away from
     the robot at this moment (it is standing in the start box).
  3. Wait for the start/stop button (the left front crash sensor).
  4. Run the task (mission.py) from the start box.
     * A press of the button at any time stops the robot.
     * A driving problem (line lost, ...) stops it with a message.
     After a stop: put the robot back in the start box (line sensors on
     START, facing north) and press the button to start a new run.

WHERE THINGS ARE
  config.py        pins, speeds, times and the task rules (all settings)
  track_map.py     the map of the table (nodes, lines, routes)
  navigation.py    driving: line following, junctions, turns, reversing
  mission.py       the task: pick up, read the magnet, deliver, go back
  sensors/, actuators/, control/   the drivers for each part of the robot
  tests/           bench tests for each part (run them on the robot)
  sim/             the PC simulator (python -m sim.run), not for the robot
"""

import machine
from utime import sleep_ms

import config
import mission
from actuators.motor import DriveBase
from control.start_stop import StartStopButton
from control.task_sensors import TaskSensors
from navigation import Navigator, StopRequested, NavigationError
from sensors.line_sensor import LineSensorArray


def wait_for_start(button):
    """Block until the start/stop button is pressed, then wait
    config.START_DELAY_MS so the hand is clear of the robot."""
    print("press the start/stop button (%s crash sensor) to start" % config.START_STOP_SENSOR)
    while not button.pressed():
        sleep_ms(5)
    print("starting in %d ms" % config.START_DELAY_MS)
    sleep_ms(config.START_DELAY_MS)


def main():
    drive = DriveBase.from_config()
    drive.stop()
    line_sensors = LineSensorArray.from_config()
    button = StartStopButton.from_config()
    tasks = TaskSensors.from_config()
    print("magnet sensor zero point: %s mV" % tasks.calibrate())
    nav = Navigator(drive, line_sensors, button, tasks)

    while True:
        wait_for_start(button)
        nav.reset_at_start()
        try:
            mission.run(nav, tasks)
        except StopRequested:
            print("stopped by the start/stop button")
        except NavigationError as error:
            print("stopped: %s" % error)
        finally:
            drive.stop()
        print("put the robot back in the start box (sensors on START, facing north)")


if __name__ == "__main__":
    try:
        main()
    finally:
        # Whatever happens (Ctrl-C, an error), switch both drive motors off:
        # otherwise the PWM outputs keep running after the program has ended.
        for pwm_pin in (config.MOTOR_LEFT_PWM_PIN, config.MOTOR_RIGHT_PWM_PIN):
            machine.PWM(machine.Pin(pwm_pin)).duty_u16(0)
        print("motors stopped")
