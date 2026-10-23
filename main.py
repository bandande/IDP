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
    # Status booleans
    turning=False   # Checks if the turning sequence is running so we just start it once
    picking_up=False
    dropping_off=False
    scantime=0
    movetime=0
    traffic=False   # Is there traffic? If so, stop all
    moving=False 
    reversing=False # Is the robot reversing or not?
    time_count=0    # How we keep track of a deadline timewise. Changed by saying time_count=now+3
    using_time=False    #Are we actually using the time right now or not?
    is_started=False    # Have we started
    count=0
    at_start=True
    
    
    # ADDED (2026-10-15): state for the fixed turns.
    left_old_line=False   # turning: True once both centre sensors have left the old line
    holdoff_until=0       # ignore junctions until now reaches this (see JUNCTION_HOLDOFF_MS)
    # ADDED (2026-10-20): True right after the robot has turned on the spot at a
    # junction to look into a full slot. Its wheels are then already on the
    # junction, so the next turn must not drive NODE_FORWARD_MS forwards first
    # (that would drive it into the rack).
    skip_forward=False
    # ADDED (2026-10-20): set when a node was counted as reached right after a
    # turn (see CLOSE_AFTER_TURN): the wheels are still short of it, so the
    # NEXT turn only drives this long first (ms). None = the normal
    # NODE_FORWARD_MS. after_turn_forward_ms holds the value until the
    # current turn has finished.

    readings=[]

    drop_off_turn_count=100
    
    robot=f.Robot()
    print(robot.position)
    f.reset_map()
    map=MAP.MAP.copy()
    # Measure elapsed time
    robot.start = utime.ticks_ms()
    # elapsed = utime.ticks_diff(utime.ticks_ms(), start)
    
    motor3 = motor.Motor(dirPin=4, PWMPin=5)  # Motor 3 is controlled from Motor Driv2 #1, which is on GP4/5
    motor4 = motor.Motor(dirPin=7, PWMPin=6)  # Motor 4 is controlled from Motor Driv2 #1, which is on GP4/5
    
    servo1, servo2=[Servo(pin) for name, pin in config.SERVO_PINS.items()]
    
    
    
    # ADDED (2026-10-15): use DRIVE_SPEED (top of this file) instead of the
    # Motor class default of 100 %. Robot.forward / left / right / backward /
    # about use these two values whenever no explicit speed is passed.
    motor3.forward_speed = c.DRIVE_SPEED
    motor3.backward_speed = c.DRIVE_SPEED
    motor4.forward_speed = c.DRIVE_SPEED
    motor4.backward_speed = c.DRIVE_SPEED

    sensors=line_sensor.LineSensorArray([20,21,19,18],True)

    # ADDED (2026-10-20): start the task sensors. calibrate() measures the
    # magnet sensor's no-magnet zero point, so it runs here, in the start box,
    # before the start button is pressed (no box / magnet near the robot).
    tasks = TaskSensors.from_config()
    print("magnet sensor zero point: %s mV" % tasks.calibrate())

    # ADDED (2026-10-15, safety): start / stop button. Nothing moves until
    # the button is pressed (see wait_for_start above).
    button = StartStopButton.from_config()
    f.wait_for_start(button)
    is_started=True
    #start running
    while True:

        f.button_check(button, is_started, robot, motor3, motor4)
        
        if is_started:
            
            count+=1
            now = utime.ticks_diff(utime.ticks_ms(), robot.start)
            f.update_sensors(sensors, robot)

            if len(robot.queue)==0:
                robot.box_count+=1
                robot.queue=["go_back","deliver","collect"]
            
            f.robot_mode_check(
                robot, 
                map, 
                motor3, 
                motor4, 
                tasks, 
                now, 
                holdoff_until, 
                skip_forward, 
                reversing, 
                time_count, 
                turning, 
                drop_off_turn_count, 
                at_start, 
                picking_up, 
                readings, 
                scantime, 
                movetime, 
                servo1, 
                servo2, 
                dropping_off
            )
            
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