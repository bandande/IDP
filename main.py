import functions
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
from sensors.line_processing import OUTER_LEFT, INNER_LEFT, INNER_RIGHT, OUTER_RIGHT
from control.start_stop import StartStopButton

# ADDED (2026-10-15): settings for the line-following bench test.
# LINE_TEST_ONLY = True: the robot ONLY follows the line. Whenever the task
#     queue is empty it is refilled with "straight" instead of
#     ["deliver", "collect"], so after every junction the robot just carries
#     on straight. (The deliver / collect planner cannot run yet: with
#     "collect" it re-plans forever without moving.) Set it to False to run
#     the full task logic again.
# DRIVE_SPEED: percent of full motor power used by Robot.forward / left /
#     right / backward when no explicit speed is given. motor.Motor defaults
#     to 100 %, far too fast for tuning the line following. Start low and
#     raise it once the robot follows the line reliably.
LINE_TEST_ONLY = True
DRIVE_SPEED = 40


def corrections(robot, motor3, motor4, motor_speed3=None, motor_speed4=None, reverse=None):
    if reverse:
        if robot.front_left_sensor == 1 and robot.front_right_sensor == 0:
            # If left front sensor does not detect and right does
            robot.queue.append(robot.mode)
            robot.mode="correction"
            robot.right(motor3,motor4,motor_speed3, motor_speed4, reverse=reverse)
            #Then turn right
            
        elif robot.front_left_sensor == 0 and robot.front_right_sensor == 1:
            # If left front sensor does not detect and right does
            robot.queue.append(robot.mode)
            robot.mode="correction"
            robot.left(motor3,motor4,motor_speed3, motor_speed4, reverse=reverse)
            #Then turn left
    else:
        if robot.front_left_sensor == 0 and robot.front_right_sensor == 1:
            # If left front sensor does not detect and right does
            robot.queue.append(robot.mode)
            robot.mode="correction"
            robot.right(motor3,motor4,motor_speed3, motor_speed4)
            #Then turn right
            
        elif robot.front_left_sensor == 1 and robot.front_right_sensor == 0:
            # If left front sensor does not detect and right does
            robot.queue.append(robot.mode)
            robot.mode="correction"
            robot.left(motor3,motor4,motor_speed3, motor_speed4)
            #Then turn left

def update_sensors(sensors, robot):
    robot.left_sensor_previous=robot.left_sensor
    robot.front_left_sensor_previous=robot.front_left_sensor
    robot.front_right_sensor_previous=robot.front_right_sensor
    robot.right_sensor_previous=robot.right_sensor
    # FIX (2026-10-15): left and right were swapped.
    # LineSensorArray.read() returns ONE 4-bit number:
    #     bit 3 = OUTER_LEFT  = Line Sensor "Edge Left"    (GP20, physical 26)
    #     bit 2 = INNER_LEFT  = Line Sensor "Centre Left"  (GP21, physical 27)
    #     bit 1 = INNER_RIGHT = Line Sensor "Centre Right" (GP19, physical 25)
    #     bit 0 = OUTER_RIGHT = Line Sensor "Edge Right"   (GP18, physical 24)
    # The old line
    #     ... = [bool(sensors.read() & (1 << i)) for i in range(4)]
    # gave bits 0, 1, 2, 3 (right to left) to left_sensor, front_left_sensor,
    # front_right_sensor, right_sensor, so a line under the Centre RIGHT
    # sensor set front_left_sensor = 1 and every correction steered AWAY from
    # the line. It also called read() four times per loop; one read is faster
    # and gives a consistent snapshot of all four sensors.
    # Values are 1 (sensor sees the white line) or 0 (black floor).
    bits = sensors.read()
    robot.left_sensor = 1 if bits & OUTER_LEFT else 0
    robot.front_left_sensor = 1 if bits & INNER_LEFT else 0
    robot.front_right_sensor = 1 if bits & INNER_RIGHT else 0
    robot.right_sensor = 1 if bits & OUTER_RIGHT else 0


def wait_for_start(button):
    """ADDED (2026-10-15, safety): block until the start/stop button is pressed.

    The button is the crash sensor named in config.START_STOP_SENSOR (default
    "left" = Crash Sensor L, physical pin 12). Because the Pico runs main.py
    by itself at power-up, waiting here stops the robot from driving off as
    soon as it is switched on. After the press it waits config.START_DELAY_MS
    so the hand that pressed the button is clear of the robot.
    """
    print("press the start/stop button (%s crash sensor) to start" % config.START_STOP_SENSOR)
    while not button.pressed():
        utime.sleep_ms(5)
    print("starting in %d ms" % config.START_DELAY_MS)
    utime.sleep_ms(config.START_DELAY_MS)


    
def main():
    turning=False   # Checks if the turning sequence is running so we just start it once
    traffic=False 
    moving=False 
    reversing=False # Is the robot reversing or not?
    time_count=0    # How we keep track of a deadline timewise. Changed by saying time_count=now+3
    using_time=False
    robot=functions.Robot()
    print(robot.position)
    functions.reset_map()
    map=MAP.MAP.copy()
    # Measure elapsed time
    start = utime.ticks_ms()
    # elapsed = utime.ticks_diff(utime.ticks_ms(), start)
    
    motor3 = motor.Motor(dirPin=4, PWMPin=5)  # Motor 3 is controlled from Motor Driv2 #1, which is on GP4/5
    motor4 = motor.Motor(dirPin=7, PWMPin=6)  # Motor 4 is controlled from Motor Driv2 #1, which is on GP4/5
    # ADDED (2026-10-15): use DRIVE_SPEED (top of this file) instead of the
    # Motor class default of 100 %. Robot.forward / left / right / backward /
    # about use these two values whenever no explicit speed is passed.
    for m in (motor3, motor4):
        m.forward_speed = DRIVE_SPEED
        m.backward_speed = DRIVE_SPEED

    sensors=line_sensor.LineSensorArray([20,21,19,18],True)

    # ADDED (2026-10-15, safety): start / stop button. Nothing moves until
    # the button is pressed (see wait_for_start above).
    button = StartStopButton.from_config()
    wait_for_start(button)

    #start running
    while True:
        # ADDED (2026-10-15, safety): a press of the start/stop button while
        # running (or the front bumping into something, since the button is a
        # crash sensor) stops both motors. The robot then waits for the next
        # press and starts again from a clean state (empty queue, mode "").
        if button.pressed():
            robot.stop(motor3, motor4)
            print("stopped by the start/stop button")
            wait_for_start(button)
            robot.mode = ""
            robot.queue = []
            turning = False

        now = utime.ticks_diff(utime.ticks_ms(), start)
        update_sensors(sensors, robot)

        if len(robot.queue)==0:
            if LINE_TEST_ONLY:
                # ADDED (2026-10-15): line-following test only (see
                # LINE_TEST_ONLY at the top): keep following the line.
                robot.queue=["straight"]
            else:
                robot.box_count+=1
                robot.queue=["deliver","collect"]
        
        
        # Action
        if robot.mode == "":
            next_item=robot.queue.pop()
            
            if isinstance(next_item, list):
                # If turning etc
                robot.mode=next_item[0]
                robot.mode_specific=next_item[1:]
                # functions.degrees_to_time(next_item[2]) # gets the count for that specific turn
                # Write some code to get or use that time
                
            elif next_item.isupper():
                robot.queue+=robot.move_to(next_item)
            else:
                robot.queue.append(next_item)
                if next_item == "collect":
                    # Plan collection point
                    # Find collection point route and enter. include "pick_up"     
                    for key in map.keys():
                        if key.startswith("GREEN"):
                            if map[key]["type"]=="DEAD_END":
                                robot.drop_off_info[0]=0
                                robot.final_objective=key
                                
                                break
                        if key.startswith("RED"):
                            if map[key]["type"]=="DEAD_END":
                                robot.drop_off_info[0]=1
                                robot.final_objective=key
                                
                                break
                    else:
                        for key in map.keys():
                            if key.startswith(("GREEN","RED")):
                                map[key]["type"]="DEAD_END"
                        robot.queue.append("collect")
                        robot.mode=""
                        
                    if not robot.mode == "":
                        robot.queue.append("pick_up")
                        robot.queue+=robot.find_route(robot.final_objective)
                        robot.mode=""
                            
                elif next_item == "deliver":
                    # Plan deliverance point
                    # Find deliverance point route and enter. include "drop_off"
                    drop_off_string = ""
                    if robot.drop_off_info[0]==0:
                        drop_off_string+="LEFT_"
                    else:
                        drop_off_string+="RIGHT_"
                    if robot.drop_off_info[1]==0:
                        drop_off_string+="BOTTOM_"
                    else:
                        drop_off_string+="TOP_"
                    drop_off_string+="DROP_"
                    
                    for drop_number in range(6):
                        if map[drop_off_string+str(drop_number)]["type"]=="DEAD_END":
                            robot.final_objective=drop_off_string+str(drop_number)                            
                            break
                    else:
                        # If all of them are full, then reset
                        for key in map.keys():
                            if key.startswith(drop_off_string):
                                map[key]["type"]="DEAD_END"
                        robot.queue.append("deliver")
                        robot.mode = ""
                        
                    if not robot.mode == "":
                        robot.queue.append("drop_off")
                        robot.queue+=robot.find_route(robot.final_objective)
                        robot.mode=""
                        
                elif next_item == "straight":
                    # FIX (2026-10-15): actually switch into "straight" mode.
                    # Before this fix the robot only called forward() and
                    # never set robot.mode, so the line corrections and the
                    # junction detection (both inside the
                    # `elif robot.mode == "straight":` branch below) never
                    # ran and the robot drove straight on blindly.
                    # The `robot.queue.append(next_item)` a few lines above
                    # put "straight" back on the queue; remove that copy
                    # again, otherwise it would be repeated forever.
                    robot.queue.pop()
                    robot.mode = "straight"
                    robot.forward(motor3,motor4)

        elif robot.mode == "straight":
            # Detection
            if ((robot.left_sensor == 1 and robot.left_sensor_previous == 0) 
                or (robot.right_sensor == 1 and robot.right_sensor_previous == 0)):
                # A junction has been detected
                
                # ##=====Can be commented out
                # robot.forward(motor3,motor4)
                # time_count=now+50
                # while now<time_count: # CHANGE THIS TO ACTUAL TIME
                #     pass# This should prevenet double triggering of junctions and allow the type of junction to be seen. This can be adjusted
                # if robot.left_sensor == 1:
                #     print("Can turn left")
                # if robot.right_sensor == 1:
                #     print("Can turn right")
                # robot.stop(motor3,motor4)
                # ##=====
                
                ##=====Can be commented out
                # THis provides the most basic logic to turning
                # if robot.left_sensor == 1:
                #     robot.queue.append(["turning", "left", 90])
                # if robot.right_sensor == 1:
                #     robot.queue.append(["turning", "right", 90])
                
                ##=====
                
                robot.previous_position=robot.position
                robot.position=robot.objective
                robot.mode="" # Moves onto next instruction (which is turning)
            # If not moving straight
            elif motor3.return_state() == 0 or motor4.return_state()==0:
                robot.forward(motor3,motor4)
                print("motor not forward")
            # Detect and make corrections
            corrections(robot,motor3,motor4, reverse=reversing)               
            
                
            
            
        elif robot.mode == "correction":
            if robot.front_left_sensor == 1 and robot.front_right_sensor == 1:
                robot.stop(motor3,motor4)
                robot.mode="" # Moves back to straight or turning
                    
        elif robot.mode == "turning":
            #robot.specific_mode is in form ["direction", degrees]
            # direction can be straight, left, right, about and degrees is the relevant degrees (90)
            # There are two ways we can do this. Either turn for a certain amount of time, or turn until fron sensors trigger
            # If straight, then go straight for a bit so that it doesn't immediately trigger junction hit again?
            
            
            if not turning:
                # Turning hasn't started, do the turns
                turning=True
                robot.mode_extra_specific="move_straight"
                if robot.mode_specific[0]=="straight":
                    # MAYBE check if will trigger junction (need to implement)
                    robot.forward(motor3,motor4)
                    robot.mode=""
                elif robot.mode_specific[0]=="left":
                    robot.forward(motor3,motor4)
                    time_count=now+2
                elif robot.mode_specific[0]=="right":
                    robot.forward(motor3,motor4)
                    time_count=now+2
                elif robot.mode_specific[0]=="about":
                    robot.forward(motor3,motor4, reverse=True)
                    reversing=True
                    time_count=now+2
                    
            elif robot.mode_extra_specific=="move_straight":
                # Make sure does not change off line                
                if now>time_count:
                    robot.mode_extra_specific="move_turning"
                    reversing=False
                    if robot.mode_specific[0]=="straight":
                        # MAYBE check if will trigger junction (need to implement)
                        robot.forward(motor3,motor4)
                        robot.mode=""
                        # Move back to straight movement
                        # initial turn
                    elif robot.mode_specific[0]=="left":
                        robot.left(motor3,motor4)
                        time_count=now+2
                    elif robot.mode_specific[0]=="right":
                        robot.right(motor3,motor4)
                        time_count=now+2
                    elif robot.mode_specific[0]=="about":
                        robot.about(motor3,motor4)
                        time_count=now+3
                    
                corrections(robot,motor3,motor4, reverse=reversing)
            elif robot.mode_extra_specific=="move_turning":
                # If we need crash detection or anything here then do it
                if now<time_count:
                    pass
                else:
                    if robot.front_left_sensor == 1 and robot.front_right_sensor == 1:
                        robot.stop(motor3,motor4)
                        robot.mode="" # Moves to next step
                        robot.direction=functions.turn_to_direction(robot.direction, robot.mode_specific[0])
                        print(robot.direction)
                        
                        turning=False
        elif robot.mode == "pick_up":
            #Do some actual grabbing stuff etc
            utime.sleep(1)
            # This should have the grabbing and so on code
            robot.drop_off_info[1]=1
            map[robot.final_objective]["type"]="COLLECTED"
            robot.mode=""
            
            pass
        elif robot.mode == "drop_off":
            #Do some actual grabbing stuff etc
            utime.sleep(2)
            # This should have the grabbing and so on code
            
            map[robot.final_objective]["type"]="FILLED"
            robot.mode=""
    




if __name__ == "__main__":
    # ADDED (2026-10-15, safety): always switch both drive motors off when the
    # program ends for ANY reason (Ctrl-C in the REPL, an exception, ...).
    # Without this the PWM outputs keep their last duty cycle and the wheels
    # keep turning after the program has stopped. GP5 and GP6 are the PWM
    # pins of motor3 (PWM1) and motor4 (PWM2); setting the duty to 0 stops them.
    try:
        main()
    finally:
        for pwm_pin in (5, 6):
            machine.PWM(machine.Pin(pwm_pin)).duty_u16(0)
        print("motors stopped")