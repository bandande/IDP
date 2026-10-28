import functions as f
import MAP
import utime
import motor
import sensors.line_sensor as line_sensor
# ADDED (2026-10-15): needed by the fixes / bench-test changes below.
#   - the bit masks name each line sensor inside LineSensorArray.read()
#   - StartStopButton uses a crash sensor as the start / stop button
#   - machine is used to switch the motors off when the program ends
#   - config holds START_STOP_SENSOR and START_DELAY_MS
import machine
import config
from actuators.servo import Servo
import re
from sensors.line_processing import OUTER_LEFT, INNER_LEFT, INNER_RIGHT, OUTER_RIGHT
from control.start_stop import StartStopButton
# ADDED (2026-10-20): the three task sensors (control/task_sensors.py):
#   - magnet sensor: north / south pole of the box at pick-up -> rack level
#   - front distance sensor: is the drop slot already full?
#   - IR receiver: traffic light at the ramp entrance (IR = green)
# Their settings (marked TODO where still unknown) are in config.py under
# "Task sensors". A sensor that fails to start is left out with a warning
# and its check uses a safe fallback.
from control.task_sensors import TaskSensors
import constants as c


def main():

    variables = f.initialise_main_variables()
    #initialise all the variables, map, tasks, robot, sensors, motors, servos
    button = variables[1]
    now = utime.ticks_diff(utime.ticks_ms(), variables[16].start)
    
    f.wait_for_start(button)
    # ADDED (2026-10-26, test build): after the start press (nothing moves
    # before it) reset the gripper the safe way and set its carry pose.
    f.gripper_start(variables[19], variables[20])
    f.grab_box(variables[19], variables[20])
    
    variables[16].queue=[
        # "go_back",
        "deliver",
        # "collect",
                            
    ]
    # variables[16].position = "JUNCTION_LEFT_BOTTOM"
    # variables[16].objective = "FUNCTION_GREEN_1"
    # variables[16].direction = "north"
    variables[16].position = "JUNCTION_GREEN_2"
    variables[16].objective = "JUNCTION_LEFT_BOTTOM_DROP_1"
    variables[16].direction = "north"
    # variables[16].position = "START"
    # variables[16].objective = "START"
    # variables[16].direction = "north"
    
    
    #start running
    utime.sleep(1)

    while True:

        variables=f.button_check(variables)
        #check for a button press
        is_started = variables[6]
        robot = variables[16]
        #fetch needed variables
        
        if is_started:
            
            now = utime.ticks_diff(utime.ticks_ms(), robot.start)
            if variables[2]%1==0:
                variables = f.update_sensors(variables)

            if len(robot.queue)==0:
                # CHANGED (2026-10-27): the extra "pick_up" at the end (= done
                # FIRST, in the start box) was a test shortcut; the pick-up is
                # now tested on its own with tests/test_pickup.py, starting
                # right after the turn into a box.
                variables[16].queue=[
                    "deliver",
                    "collect",
                                     
                ]
                
            variables, now = f.robot_mode_check(variables, now)
            # Action
            




if __name__ == "__main__":

    try:
        main()
    finally:
        for pwm_pin in (5, 6):
            machine.PWM(machine.Pin(pwm_pin)).duty_u16(0)
        for servo in [Servo(pin) for name, pin in config.SERVO_PINS.items()]:
            servo.set_angle(0)
            servo.off()

        print("motors stopped")