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
from actuators.servo import Servo
import re
from numpy import mean
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

# DRIVE_SPEED: percent of full motor power used by Robot.forward / left /
#     right / backward when no explicit speed is given (motor.Motor's own
#     default is 100 %).
# CHANGED (2026-10-20): LINE_TEST_ONLY and the line-test route were removed
# from main.py (as requested). For line-following tests on the bench use
# tests/test_line_follow.py.
DRIVE_SPEED = 60
# DRIVE_SPEED = 100


# Turning and junction settings. All times are in MILLISECONDS (`now` in
# main() counts milliseconds).
# NODE_FORWARD_MS: the line sensors sit in front of the wheels, so when the
#     sensors reach a junction the wheels are still short of it. Before
#     turning on the spot the robot drives on for this long so that the
#     WHEELS are on the junction. Roughly (distance from the line sensors to
#     the wheel axle) / (driving speed).
# TURN_SPEED: motor power (%) of both wheels while turning on the spot.
# TURN_MIN_MS: a turn never finishes earlier than this.
# START_TURN_MIN_MS: ADDED (2026-10-20): the same for the first turn at the
#     start (the start box lines have to be passed). It replaces the
#     "TURN_MIN_MS=1700" that used to be set inside main(), see the turning
#     code.
# JUNCTION_HOLDOFF_MS: CHANGED (2026-10-20): after a junction has been
#     detected, ignore new junctions for this long, so that the same
#     junction is not counted twice while the outer sensor slides off its
#     side line (it can flicker black-white-black). NOT used after a turn:
#     then the sensors are far from the old junction, and a hold-off could
#     make the robot miss a junction only a few cm ahead.
# LONG_PRESS_MS: ADDED (2026-10-20): the start/stop button has two kinds of
#     stop, see "two kinds of stop" in main(): a short press pauses, a press
#     held at least this long stops and resets.
# NODE_FORWARD_MS = 1600
# TURN_MIN_MS = 1800

NODE_FORWARD_MS = 1600
TURN_MIN_MS = 1200
START_TURN_MIN_MS = 1700
TURN_SPEED = 60


# NODE_FORWARD_MS = 800
# TURN_MIN_MS = 650
# TURN_SPEED = 100

TURN_TIMEOUT_MS = 4000
LINE_LOST_MS = 150
REVERSE_TIMEOUT_MS = 5000
JUNCTION_HOLDOFF_MS = 400
LONG_PRESS_MS = 1500

DISTANCE_TO_SCAN_PICK_UP = 150
DISTANCE_TO_PICK_UP = 20
SCAN_TURN_TIME = 100

DISTANCE_TO_DROP_OFF_SCAN = 3000
DISTANCE_TO_DROP_POINT = 3000




def corrections(robot, motor3, motor4, motor_speed3=TURN_SPEED, motor_speed4=TURN_SPEED, reverse=None, time_count=0):
    if reverse:
        if robot.front_left_sensor == 0 and robot.front_right_sensor == 1:
            # If left front sensor does not detect and right does
            robot.queue.append(robot.mode)
            
            robot.mode="correction"
            robot.right(motor3,motor4,motor_speed3, motor_speed4, reverse=reverse)
            now = utime.ticks_diff(utime.ticks_ms(), robot.start)
            time_count=time_count-now
            #Then turn right
            
        elif robot.front_left_sensor == 1 and robot.front_right_sensor == 0:
            # If left front sensor does not detect and right does
            robot.queue.append(robot.mode)
            robot.mode="correction"
            robot.left(motor3,motor4,motor_speed3, motor_speed4, reverse=reverse)
            now = utime.ticks_diff(utime.ticks_ms(), robot.start)
            time_count=time_count-now
            #Then turn left
    else:
        if robot.front_left_sensor == 0 and robot.front_right_sensor == 1:
            # If left front sensor does not detect and right does
            robot.queue.append(robot.mode)
            robot.mode="correction"
            robot.right(motor3,motor4,motor_speed3, motor_speed4)
            now = utime.ticks_diff(utime.ticks_ms(), robot.start)
            time_count=time_count-now
            #Then turn right
            
        elif robot.front_left_sensor == 1 and robot.front_right_sensor == 0:
            # If left front sensor does not detect and right does
            robot.queue.append(robot.mode)
            robot.mode="correction"
            robot.left(motor3,motor4,motor_speed3, motor_speed4)
            now = utime.ticks_diff(utime.ticks_ms(), robot.start)
            time_count=time_count-now
            #Then turn left

def update_sensors(sensors, robot):
    robot.left_sensor_previous=robot.left_sensor
    robot.front_left_sensor_previous=robot.front_left_sensor
    robot.front_right_sensor_previous=robot.front_right_sensor
    robot.right_sensor_previous=robot.right_sensor
    
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
        # print("hello")
    print("starting in %d ms" % config.START_DELAY_MS)
    utime.sleep_ms(config.START_DELAY_MS)





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

    drop_off_turn_count=100
    
    robot=functions.Robot()
    print(robot.position)
    functions.reset_map()
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
    motor3.forward_speed = DRIVE_SPEED
    motor3.backward_speed = DRIVE_SPEED
    motor4.forward_speed = DRIVE_SPEED
    motor4.backward_speed = DRIVE_SPEED

    sensors=line_sensor.LineSensorArray([20,21,19,18],True)

    # ADDED (2026-10-20): start the task sensors. calibrate() measures the
    # magnet sensor's no-magnet zero point, so it runs here, in the start box,
    # before the start button is pressed (no box / magnet near the robot).
    tasks = TaskSensors.from_config()
    print("magnet sensor zero point: %s mV" % tasks.calibrate())

    # ADDED (2026-10-15, safety): start / stop button. Nothing moves until
    # the button is pressed (see wait_for_start above).
    button = StartStopButton.from_config()
    wait_for_start(button)
    is_started=True
    #start running
    while True:
        # FIX (2026-10-20): read the button ONCE per loop. button.pressed()
        # returns True only once for each press. The old code called it twice:
        #     if button.pressed() and not is_started:    (1)
        #     ...
        #         if button.pressed():                   (2)
        # While the robot was running, call (1) used up the press and then
        # threw it away (because is_started was True), so call (2) almost
        # never saw it: that is why the button sometimes did not stop the
        # robot.
        pressed = button.pressed()
        if pressed and not is_started:
            is_started=True
            print("Started")
            pressed = False          # this press only started the robot

        if is_started:
            # CHANGED (2026-10-20): two kinds of stop (the button is the left
            # front crash sensor, so bumping into something also stops):
            #   * SHORT press = PAUSE: the motors stop and the robot keeps
            #     everything (position, queue, mode, timers). The next press
            #     continues exactly where it stopped.
            #   * LONG press (held for LONG_PRESS_MS) = STOP AND RESET: the
            #     robot forgets its route and position (robot.reset(): START,
            #     facing north). Put it back in the start box, then press to
            #     start again. What it has collected / delivered so far stays
            #     marked in the map.
            # (The old 2 x utime.sleep(1) after a stop were removed: a press
            # during them was remembered and restarted the robot at once.)
            if pressed:
                # Remember what both motors were doing, so a pause can resume.
                saved_motors = [(m.mDir.value(), m.pwm.duty_u16()) for m in (motor3, motor4)]
                robot.stop(motor3, motor4)
                pause_start = utime.ticks_ms()
                long_press = False
                while button.held():
                    button.pressed()          # keeps the button's debouncer up to date
                    if utime.ticks_diff(utime.ticks_ms(), pause_start) >= LONG_PRESS_MS:
                        long_press = True
                        break
                    utime.sleep_ms(10)
                if long_press:
                    print("STOP AND RESET: put the robot back in the start box, then press to start")
                    while button.held():      # wait until the button is let go
                        button.pressed()
                        utime.sleep_ms(10)
                    robot.reset()
                    turning=False
                    reversing=False
                    skip_forward=False
                    holdoff_until=0
                    at_start=True
                    wait_for_start(button)
                    robot.start = utime.ticks_ms()
                else:
                    print("PAUSED: press the button again to continue from here")
                    while not button.pressed():
                        utime.sleep_ms(5)
                    utime.sleep_ms(config.START_DELAY_MS)   # hand clear of the robot
                    # Shift the time reference by the length of the pause, so
                    # `now` does not jump and every running timer (time_count)
                    # still has the time it had left before the pause.
                    paused_ms = utime.ticks_diff(utime.ticks_ms(), pause_start)
                    robot.start = utime.ticks_add(robot.start, paused_ms)
                    for m, (direction, duty) in zip((motor3, motor4), saved_motors):
                        m.mDir.value(direction)
                        m.pwm.duty_u16(duty)
                    print("continuing after a %d ms pause" % paused_ms)
                continue

            count+=1
            now = utime.ticks_diff(utime.ticks_ms(), robot.start)
            update_sensors(sensors, robot)

            if len(robot.queue)==0:
                robot.box_count+=1
                robot.queue=["go_back","deliver","collect"]
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
                    print("destination:"+next_item)
                    print("current:"+robot.position)

                    robot.queue+=robot.move_to(next_item)
                    
                    # If needs to reverse (dead_end), then if coming from drop off and going to
                    if ((next_item.startswith("JUNCTION_GREEN_") or next_item.startswith("JUNCTION_RED_")) 
                        and (robot.position.startswith("GREEN_") or robot.position.startswith("RED_"))):
                        robot.queue = robot.queue[:-2]+["reverse_straight"]
                        print(robot.queue)
                    
                    # Prevent moving forwards when picking up the box.
                    if ((next_item.startswith("GREEN_") or next_item.startswith("RED_")) 
                        and (robot.position.startswith("JUNCTION_GREEN_") or robot.position.startswith("JUNCTION_RED_"))):
                        # Remove straight
                        robot.queue = robot.queue[:-2]+[robot.queue[-1]]
                        print(robot.queue)
                    
                    # Prevent moving forwards when picking up the box.
                    if ((bool(re.match(r"^JUNCTION.*DROP",next_item))) 
                        and bool(re.match(r"^(?!.*JUNCTION).*DROP",robot.position))):
                        # Remove straight
                        robot.queue = robot.queue[:-2]+[robot.queue[-1]]
                        print(robot.queue)

                    # Stops errors with turning because line spacing is too small
                    # If next is a junction and currently at a junction and came from a drop, then add a small reverse
                    if ((bool(re.match(r"^JUNCTION.*DROP",next_item))) 
                        and (bool(re.match(r"^JUNCTION.*DROP",robot.position))) 
                        and bool(re.match(r"^(?!.*JUNCTION).*DROP",robot.previous_position))):
                        # Remove straight
                        robot.queue = robot.queue[:-1]+["reverse_straight"]+[robot.queue[-1]]
                        print(robot.queue)

                    # ADDED (2026-10-20): traffic light. If the stretch from
                    # here to next_item is in config.TRAFFIC_LIGHT_EDGES (the
                    # ramp entrance: START -> TOP_CENTRE), put "wait_light"
                    # between the turn and the drive. move_to() ended the queue
                    # with [..., "straight", turn] and the queue is used from
                    # the END, so this becomes: turn -> wait_light -> straight.
                    if (robot.position, next_item) in config.TRAFFIC_LIGHT_EDGES:
                        robot.queue = robot.queue[:-1] + ["wait_light", robot.queue[-1]]

    

                else:
                    robot.mode=next_item
                    if next_item == "collect":
                        # Plan collection point
                        # Find collection point route and enter. include "pick_up"     
                        for key in map.keys():
                            
                            if key.startswith("GREEN_"):
                                if map[key]["type"]=="DEAD_END":
                                    robot.drop_off_info[0]=0
                                    robot.final_objective=key
                                
                                    break
                            if key.startswith("RED_"):
                                if map[key]["type"]=="DEAD_END":
                                    robot.drop_off_info[0]=1
                                    robot.final_objective=key
                                
                                    break
                        else:
                            for key in map.keys():
                                if key.startswith(("GREEN_","RED_")):
                                    map[key]["type"]="DEAD_END"
                            robot.queue.append("collect")
                            robot.mode=""
                        
                        if not robot.mode == "":
                            robot.queue.append("pick_up")
                            print(robot.find_route(robot.final_objective))
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
                    
                        for drop_number in range(1,7):
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
                            print(robot.find_route(robot.final_objective))
                            print("FINAL_OBJECTIVE: " + robot.final_objective)
                            robot.queue+=robot.find_route(robot.final_objective)
                            robot.mode=""
                                                
                            
                    elif next_item == "go_back":
                        robot.final_objective="START"
                        robot.queue.append("stop")
                        robot.find_route(robot.final_objective)
                        robot.queue+=robot.find_route(robot.final_objective)
                        robot.mode=""
                        
                    elif next_item == "straight":

                        robot.forward(motor3,motor4)
                    elif next_item == "reverse_straight":
                        print(robot.mode)
                        reversing=True
                        robot.forward(motor3,motor4, reverse=reversing)
                        robot.mode="straight"
                    elif next_item == "wait_light":
                        # ADDED (2026-10-20): stop in front of the traffic light
                        # and watch it; the "wait_light" mode below goes on
                        # when it is green.
                        robot.stop(motor3, motor4)
                        tasks.light_reset()
                        print("waiting for the traffic light")
            
            elif robot.mode == "straight":

                # Detection
                # CHANGED (2026-10-20): junctions are ignored until
                # holdoff_until (set when a junction is detected, see
                # JUNCTION_HOLDOFF_MS at the top; not set after a turn).
                if ((robot.left_sensor == 1 and robot.left_sensor_previous == 0)
                    or (robot.right_sensor == 1 and robot.right_sensor_previous == 0)) \
                        and now >= holdoff_until:
                    # A junction has been detected
                
                    print("JUNCTION DETECTED")
                    robot.previous_position=robot.position
                    print(robot.position)

                    robot.position=robot.objective
                    # print("ROBOT_OBJECTIVE"+robot.objective)
                    
                    # In case it wants to reverse
                    if reversing:
                        utime.sleep_ms(100)
                        reversing=False
                    robot.mode="" # Moves onto next instruction (which is turning)
                    holdoff_until = now + JUNCTION_HOLDOFF_MS   # ADDED (2026-10-20)
                # If not moving straight
                elif motor3.return_state() == 0 or motor4.return_state()==0:
                    robot.forward(motor3,motor4, reverse=reversing)
                    print("motor not forward")
                if not reversing:
                    corrections(robot,motor3,motor4, reverse=reversing)
            
                
            
            
            elif robot.mode == "correction":

                if robot.front_left_sensor == 1 and robot.front_right_sensor == 1:
                    robot.stop(motor3,motor4)
                    robot.mode="" # Moves back to straight or turning
                    now = utime.ticks_diff(utime.ticks_ms(), robot.start)
                    time_count=now+time_count
                # Detect if there is a dead_end. If there is, then stop going straight.
                elif robot.front_left_sensor == 0 and robot.front_right_sensor == 0 and robot.queue[-1]=="straight":
                    #' I can specify further with more ifs
                    robot.stop(motor3,motor4)
                    robot.previous_position=robot.position
                    print(robot.position)

                    robot.position=robot.objective
                    robot.queue.pop()
                    robot.mode=""
                    
                    now = utime.ticks_diff(utime.ticks_ms(), robot.start)
                    time_count=now+time_count

            elif robot.mode == "turning":
                #robot.specific_mode is in form ["direction", degrees]
                # direction can be straight, left, right, about and degrees is the relevant degrees (90)
                # There are two ways we can do this. Either turn for a certain amount of time, or turn until fron sensors trigger
                # If straight, then go straight for a bit so that it doesn't immediately trigger junction hit again?
            
                # CHANGED (2026-10-20): skip_forward (see the top of main()): no
                # driving forwards before this turn, the wheels are already on
                # the junction.
                if not turning:
                    # Turning hasn't started, do the turns
                    turning=True
                    robot.mode_extra_specific="move_straight"
                    
                    
                    if skip_forward:
                        forward_ms = 0
                    else:
                        forward_ms = NODE_FORWARD_MS
                    skip_forward = False
                    drop_off_turn_count+=1
                    if robot.mode_specific[0]=="straight":
                        # MAYBE check if will trigger junction (need to implement)
                        robot.forward(motor3,motor4)
                        time_count=now+int(forward_ms/4)
                    elif robot.mode_specific[0]=="left":
                        robot.forward(motor3,motor4)
                        time_count=now+forward_ms
                    elif robot.mode_specific[0]=="right":
                        robot.forward(motor3,motor4)
                        time_count=now+forward_ms
                    elif robot.mode_specific[0]=="about":
                        robot.forward(motor3,motor4, reverse=True)
                        reversing=True
                        time_count=now+int(forward_ms/8)
                    

                elif robot.mode_extra_specific=="move_straight":
                    # Make sure does not change off line
                    if now>=time_count:
                        robot.mode_extra_specific="move_turning"
                        reversing=False
                        left_old_line=False

                        # CHANGED (2026-10-20): TURN_MIN_MS stays where it is (top
                        # of this file) and is no longer changed here. Assigning
                        # to it inside main() made Python treat TURN_MIN_MS as a
                        # LOCAL variable of main() everywhere in main(), which
                        # only worked by luck. turn_min is the minimum turn time
                        # for THIS turn: START_TURN_MIN_MS for the first turn at
                        # the start (exact turn especially for the start).
                        turn_min = START_TURN_MIN_MS if at_start else TURN_MIN_MS
                        
                        
                        if robot.mode_specific[0]=="straight":
                            # MAYBE check if will trigger junction (need to implement)
                            robot.forward(motor3,motor4,reverse=reversing)
                            robot.mode=""
                            turning=False
                            # Move back to straight movement
                            # initial turn
                        elif robot.mode_specific[0]=="left":
                            robot.spin(motor3, motor4, "left", TURN_SPEED)
                            time_count=now+turn_min
                        elif robot.mode_specific[0]=="right":
                            robot.spin(motor3, motor4, "right", TURN_SPEED)
                            time_count=now+turn_min
                        elif robot.mode_specific[0]=="about":
                            robot.about(motor3,motor4)
                            time_count=now+turn_min*2

                        at_start=False


                elif robot.mode_extra_specific=="move_turning":
                    # If we need crash detection or anything here then do it
                    if robot.front_left_sensor == 0 and robot.front_right_sensor == 0:
                        left_old_line = True
                    if now<time_count or not left_old_line:
                        pass
                    else:
                        if robot.front_left_sensor == 1 and robot.front_right_sensor == 1:
                            robot.stop(motor3,motor4)
                            robot.mode="" # Moves to next step
                            robot.direction=functions.turn_to_direction(robot.direction, robot.mode_specific[0])
                            print(robot.direction)

                            turning=False
                            # CHANGED (2026-10-20): no junction hold-off after a
                            # turn (see JUNCTION_HOLDOFF_MS at the top).
                            holdoff_until = 0
                            
                            
                            
            elif robot.mode == "pick_up":
                
                if not picking_up:
                    #Do some actual grabbing stuff etc
                    robot.previous_position=robot.position
                    robot.position=robot.objective
                    print(robot.position)
                    utime.sleep(1)
                    print("GRABBING STUFF")
                    # Turning hasn't started, do the turns
                    picking_up=True
                    robot.mode_extra_specific="scan"
                    
                    
                    readings = []
                    time_count=now+SCAN_TURN_TIME
                    scantime=now
                    robot.spin(motor3, motor4, "left", TURN_SPEED)
                    

                elif robot.mode_extra_specific=="scan":
                    if now>=time_count and motor3.return_state()==-1:
                        time_count+=SCAN_TURN_TIME*2
                        robot.spin(motor3, motor4, "right", TURN_SPEED/2)
                    elif now>=time_count and motor3.return_state()==1:
                        time_count+=SCAN_TURN_TIME*2
                        robot.spin(motor3, motor4, "left", TURN_SPEED/2)
                    
                    if now%5==0:
                        if tasks.front_tof.poll():
                            d = tasks.front_tof.get_distance()
                            if d is not None:
                                readings.append(d)
                        mean_scan=mean(readings[-10:])
                        if mean_scan<DISTANCE_TO_SCAN_PICK_UP:
                            scantime=now-scantime
                            robot.mode_extra_specific="move_straight"
                            robot.forward(motor3,motor4,DRIVE_SPEED/2,DRIVE_SPEED/2)
                            movetime=now
                            time_count=now+500
                            
                            
                elif robot.mode_extra_specific=="move_straight":
                    if now>=time_count and motor3.return_state()==1:
                        time_count+=500
                        robot.forward(motor3,motor4,DRIVE_SPEED/2, DRIVE_SPEED/2, reverse=True)
                    elif now>=time_count and motor3.return_state()==-1:
                        # Go back to scan if see nothing
                        robot.mode_extra_specific="scan"
                        robot.spin(motor3, motor4, "left", TURN_SPEED/2)
                    
                    if tasks.front_tof.poll():
                        d = tasks.front_tof.get_distance()
                        if d is not None:
                            readings.append(d)
                    mean_scan=mean(readings[-10:])
                    if mean_scan<DISTANCE_TO_PICK_UP:
                        movetime=now-movetime
                        robot.mode_extra_specific="pick_up_grab_start"
                        robot.stop(motor3,motor4)
                
                elif robot.mode_extra_specific=="pick_up_grab_start":
                    pole = tasks.read_pole()
                    if pole:
                        level = tasks.level_for_pole(pole)
                        robot.drop_off_info[1] = 1 if level == "TOP" else 0
                        print("box magnet: %s (strongest field %s mV) -> %s level" % (
                        pole if pole else "NOT FOUND", tasks.last_field_mv, level))
                    else:
                        # In case it can't see it
                        print("CANT SEE")
                        robot.mode="move_straight"
                        robot.forward(motor3,motor4,DRIVE_SPEED/2)
                        time_count=now+500
                        continue
                    # Engage servos
                    # Do the whole servo grabbing motion
                    utime.sleep_ms(100)
                    robot.mode_extra_specific="return_straight"
                    time_count=now+movetime%500
                    robot.forward(motor3,motor4, DRIVE_SPEED/2, reverse=True)
                    
                elif robot.mode_extra_specific=="return_straight":
                    if now>=time_count:
                        robot.mode_extra_specific="return turn"
                        time_count=now+scantime%SCAN_TURN_TIME
                        if scantime%(SCAN_TURN_TIME*4)+SCAN_TURN_TIME<SCAN_TURN_TIME*2:
                            robot.spin(motor3,motor4,"right", TURN_SPEED/2)
                        else:
                            robot.spin(motor3,motor4,"left", TURN_SPEED/2)
                elif robot.mode_extra_specific=="return_turn":  
                    if now>=time_count:
                        map[robot.final_objective]["type"]="COLLECTED"
                        robot.mode=""
                        robot.has_box=True
                        picking_up=False
                
                # This should have the grabbing and so on code
                # robot.drop_off_info[1]=0
                # CHANGED (2026-10-20): the fixed 0 above was a placeholder for
                # the rack level. The level now comes from the magnet in the
                # box: read_pole() watches the magnet sensor for
                # config.MAG_READ_MS and returns "north", "south" or None;
                # config.MAG_POLE_LEVEL turns that into "TOP" / "BOTTOM"
                # (MAG_NO_POLE_LEVEL if no magnet was found). "deliver" reads
                # drop_off_info[1]: 0 -> BOTTOM_ slots, 1 -> TOP_ slots.
                
                
                    
            
                
            elif robot.mode == "drop_off":
                #Do some actual dropping off stuff etc
                utime.sleep(2)
                print("DROPPING STUFF")
                # This should have the dropping off and so on code
                if not dropping_off:
                    #Do some actual grabbing stuff etc
                    robot.previous_position=robot.position
                    robot.position=robot.objective
                    print(robot.position)
                    utime.sleep(1)
                    print("DROPPING STUFF")
                    # Turning hasn't started, do the turns
                    dropping_off=True
                    robot.mode_extra_specific="move_straight_and_detect"
                    
                    robot.forward(motor3,motor4)
                    # If at top, then move very slightly less
                    if robot.drop_off_info[1]==0:
                        drop_off_distance=DISTANCE_TO_DROP_OFF_SCAN
                    else:
                        drop_off_distance=DISTANCE_TO_DROP_OFF_SCAN-200
                        
                    time_count=now+drop_off_distance
                elif robot.mode_extra_specific=="move_straight_and_detect":
                    if now>=time_count:
                        robot.stop(motor3,motor4)
                        robot.mode_extra_specific="move_turning"
                        drop_off_turn_count=0
                    
                    corrections(robot, motor3, motor4, time_count=time_count)
                elif robot.mode_extra_specific=="move_turning":
                    # If this is coming from drop_off point
                    if drop_off_turn_count==0:
                        skip_forward=True
                        robot.queue+=["drop_off", ["turning", "about", 90]]
                        robot.mode=""
                    elif drop_off_turn_count==1:
                        # Use sensor to detect if box is there or not
                        
                        empty=True
                        # If the space is empty
                        if empty==True:
                            skip_forward=True
                            robot.queue+=["drop_off", ["turning", "about", 90]]
                            robot.mode=""
                        else:
                            map[robot.final_objective]["type"]="FILLED"
                            robot.queue+=["deliver"]
                            robot.mode=""
                            dropping_off=False
                    elif drop_off_turn_count==2:
                        robot.mode_extra_specific="move_and_drop_off"
                        robot.forward(motor3,motor4)
                        time_count=now+DISTANCE_TO_DROP_POINT
                elif robot.mode_extra_specific=="move_and_drop_off":
                    if now>=time_count:
                        robot.stop(motor3, motor4)
                        # Do the servo motions and moving, using sleep as well for moving forwards as 
                        # this needs to be hugely precise
                        # Drop off
                        print("EVERYTHING IS DROPPED OFF. COUNT: "+str(robot.box_count))
                        robot.box_count+=1
                        robot.has_box=False
                        
                        map[robot.final_objective]["type"]="FILLED"
                        
                        robot.mode=""
                    corrections(robot, motor3, motor4, time_count=time_count)

                
            elif robot.mode == "wait_light":
                # ADDED (2026-10-20): stopped in front of the traffic light (see
                # "wait_light" above). Drive on when the IR receiver reports
                # green (IR beacon seen without a break for
                # config.TRAFFIC_IR_CONFIRM_MS). No time limit.
                if tasks.light_update() == "green":
                    print("traffic light green - go (waited %d ms)" % tasks.light_waited_ms())
                    robot.mode=""

            elif robot.mode == "stop":
                is_started=False
                robot.stop(motor3, motor4)
                print("REACHED START")
    




if __name__ == "__main__":

    try:
        main()
    finally:
        for pwm_pin in (5, 6):
            machine.PWM(machine.Pin(pwm_pin)).duty_u16(0)
        print("motors stopped")