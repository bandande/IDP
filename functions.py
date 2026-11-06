import constants as c
from collections import deque
from motor import Motor
from sensors.line_processing import OUTER_LEFT, INNER_LEFT, INNER_RIGHT, OUTER_RIGHT
import utime
import config
from control.task_sensors import TaskSensors
from actuators.leds import StatusLeds
import re
import motor
from actuators.servo import Servo
import MAP
import sensors.line_sensor as line_sensor
from control.start_stop import StartStopButton
from control.pickup import PickUp
from actuators.gripper import gripper_carry, gripper_start, gripper_safe, release_box, gripper_lift


def turn_to_direction(current_direction, turn):
    directions = ["north", "east", "south", "west"]

    if current_direction not in directions:
        raise ValueError(f"Invalid direction: {current_direction}")

    if turn == "left":
        change = -1
    elif turn == "right":
        change = 1
    elif turn == "about":
        change = 2
    elif turn == "straight":
        change = 0
    else:
        raise ValueError(f"Invalid turn: {turn}")

    return directions[(directions.index(current_direction) + change) % 4]

def direction_to_turn(current_direction, target_direction):
    directions = ["north", "east", "south", "west"]

    if current_direction not in directions:
        raise ValueError(f"Invalid current direction: {current_direction}")

    if target_direction not in directions:
        raise ValueError(f"Invalid target direction: {target_direction}")

    difference = (
        directions.index(target_direction)
        - directions.index(current_direction)
    ) % 4

    if difference == 0:
        return "straight"
    elif difference == 1:
        return "right"
    elif difference == 2:
        return "about"
    else:
        return "left"

def reset_map(old_phrases=("COLLECTED", "OCCUPIED")):
    for node in MAP.MAP.values():
        if node.get("type") in old_phrases:
            node["type"] = "DEAD_END"


    

class Robot:
    
    def reset(self):
        

        self.previous_position = "START"
        self.position = "START"
        self.direction = "north"
        self.objective = ""
        self.final_objective = ""
        self.mode = ""
        self.mode_store = ""
        self.mode_specific = []
        self.mode_extra_specific = ""
        # mode=(collecting, turning, stopping, straight, dropping)
        
        self.queue: list[str | list]= [] # Macro is the largest. It is as follows- first element is the name, next elements are instructions
        
        
        self.left_sensor = True
        self.right_sensor = True
        self.front_left_sensor = True
        self.front_right_sensor = True
        
        self.left_sensor_previous = True
        self.right_sensor_previous = True
        self.front_left_sensor_previous = True
        self.front_right_sensor_previous = True
        
        self.has_box = False
        self.box_count = 0
        
        self.motor_left=False
        self.motor_right=False
        
        self.drop_off_info=[0,1,0]
        
        self.start=0
        
        self.baseline_data={}
    
    def __init__(self):
        self.reset()
        
        
    def forward(self, motor3, motor4, motor3_speed=None, motor4_speed=None, reverse=False):
        """motor3 is left. motor4 is right
        """
        if reverse:
            if not motor3_speed:
                motor3_speed=motor3.backward_speed
            if not motor4_speed:
                motor4_speed=motor4.backward_speed
            self.motor_left=True
            self.motor_right=True
            motor3.Reverse(motor3_speed)
            motor4.Reverse(motor4_speed)
        else:
            if not motor3_speed:
                motor3_speed=motor3.forward_speed
            if not motor4_speed:
                motor4_speed=motor4.forward_speed
            self.motor_left=True
            self.motor_right=True
            motor3.Forward(motor3_speed)
            motor4.Forward(motor4_speed)
            
    def left(self, motor3, motor4, motor3_speed=None, motor4_speed=None, reverse=False):
            """motor3 is left. motor4 is right
            """
            if reverse:
                if not motor3_speed:
                    motor3_speed=motor3.backward_speed
                if not motor4_speed:
                    motor4_speed=motor4.backward_speed
                self.motor_left=True
                self.motor_right=False
                motor3.Reverse(motor3_speed)
                motor4.off()
            else:
                if not motor3_speed:
                    motor3_speed=motor3.forward_speed
                if not motor4_speed:
                    motor4_speed=motor4.forward_speed
                self.motor_left=False
                self.motor_right=True
                motor3.Forward(motor3_speed)
                motor4.Forward(motor4_speed)
                
    def right(self, motor3, motor4, motor3_speed=None, motor4_speed=None, reverse=False):
            """motor3 is left. motor4 is right
            """
            if reverse:
                if not motor3_speed:
                    motor3_speed=motor3.backward_speed
                if not motor4_speed:
                    motor4_speed=motor4.backward_speed
                self.motor_left=False
                self.motor_right=True
                motor3.off()
                motor4.Reverse(motor4_speed)
            else:
                if not motor3_speed:
                    motor3_speed=motor3.forward_speed
                if not motor4_speed:
                    motor4_speed=motor4.forward_speed
                self.motor_left=True
                self.motor_right=False
                motor3.Forward(motor3_speed)
                motor4.Forward(motor4_speed)
    def about(self, motor3, motor4, motor3_speed=None, motor4_speed=None, direction="right"):
            """motor3 is left. motor4 is right
            direction="right" (the default) turns round clockwise. direction="left" is the exact mirror image
            (anticlockwise): the wheels swap roles and speeds. Used on the top
            level, where the U-turn must go towards the open side because of
            the railings."""
            if not motor3_speed:
                motor3_speed=motor3.forward_speed
            if not motor4_speed:
                motor4_speed=motor4.backward_speed
            self.motor_left=True
            self.motor_right=True
            if direction == "left":
                # mirror image of the lines below
                motor3.Reverse(motor4_speed)
                motor4.Forward(motor3_speed)
            else:
                motor3.Forward(motor3_speed)
                motor4.Reverse(motor4_speed)
    
    def stop(self, motor3, motor4, motor3_speed=None, motor4_speed=None):
                """motor3 is left. motor4 is right
                """
                if not motor3_speed:
                    motor3_speed=motor3.forward_speed
                if not motor4_speed:
                    motor4_speed=motor4.forward_speed
                self.motor_left=False
                self.motor_right=False
                motor3.off()
                motor4.off()
    
    def spin(self, motor3, motor4, direction, speed):
        """Turn on the spot, "left" or "right".

        One wheel drives forwards and the other backwards at the same power, so
        the robot rotates about the middle of its wheel axle and stays on the
        junction.
        motor3 is the left wheel, motor4 the right wheel."""
        if direction == "left":
            motor3.Reverse(speed)
            motor4.Forward(speed)
        else:
            motor3.Forward(speed)
            motor4.Reverse(speed)
        self.motor_left = True
        self.motor_right = True

    def pivot(self, motor3, motor4, direction, speed):
        """Turn about ONE wheel, "left" or
        "right". The wheel on the inside of the turn stands still and the
        outside wheel drives forwards, so the robot rotates about the inside
        wheel. Unlike spin(), the robot then needs to drive on much less
        before the turn (see c.PIVOT_FORWARD_MS), so a box in the gripper
        stays further from the table edge in front of it.
        motor3 is the left wheel, motor4 the right wheel."""
        if direction == "left":
            motor3.off()
            motor4.Forward(speed)
            self.motor_left = False
            self.motor_right = True
        else:
            motor3.Forward(speed)
            motor4.off()
            self.motor_left = True
            self.motor_right = False


    from collections import deque

    def order_colours(self, targets = ["GREEN_1", "GREEN_2", "RED_1", "RED_2"]):
        start = self.position
        on_left_top=False

        # Find shortest distances from the robot's position
        distances = {start: 0}
        queue = deque([start], 100)

        while queue:
            current = queue.popleft()

            for direction in ("north", "south", "east", "west"):
                neighbour = MAP.MAP[current].get(direction)

                if neighbour is not None and neighbour not in distances:
                    distances[neighbour] = distances[current] + 1
                    queue.append(neighbour)
        
        # Apply a small preference when the robot is at the top
        if self.drop_off_info[0]==0 and self.drop_off_info[1]==1:
            on_left_top=True

        def score(target):
            distance = distances.get(target, float("inf"))

            if on_left_top and target.startswith("GREEN"):
                distance -= 2.25
            elif not on_left_top and target.startswith("RED"):
                distance -= 2.25

            return distance

        # Sort targets by distance from the robot
        return sorted(
            targets,
            key=score
        )
        
    
    def find_route(self, location):
        # Check that both locations exist
        if self.position not in MAP.MAP or location not in MAP.MAP:
            return ["Error position and location not found"]

        # Queue of locations to explore
        queue = deque((), len(MAP.MAP))
        queue.append(self.position)

        # Remember how each location was reached
        previous = {self.position: None}

        while queue:
            current = queue.popleft()

            # Target found: reconstruct the path
            if current == location:
                path = []

                while current is not None:
                    path.append(current)
                    current = previous[current]
                return path[:-1]

            # Explore all connected neighbours
            for direction in ("north", "south", "east", "west"):
                neighbour = MAP.MAP[current].get(direction)

                if neighbour is None:
                    continue

                if neighbour not in MAP.MAP:
                    continue

                if neighbour not in previous:
                    previous[neighbour] = current
                    queue.append(neighbour)

        # No route exists
        return ["Error no route found"]
    
    def move_to(self, location):
        # From one point to adjacent point
        if MAP.MAP.get(self.position) and MAP.MAP.get(location):
            if self.position == location:
                return []
            final_direction = None
            for direction in ("north", "east", "south", "west"):
                if MAP.MAP[self.position].get(direction) == location:
                    final_direction = direction
                    break
            if final_direction is None:
                raise ValueError("move_to: %s is not next to %s" % (location, self.position))
            turning=direction_to_turn(self.direction, final_direction)
        else:
            print("error, location or own position not found")
        
        
        self.objective = location
        # If it is backwards, have a negative on the 90.
        return ["straight", ["turning", turning, 90]]
    
    
def corrections(robot, motor3, motor4, motor_speed3=c.TURN_SPEED, motor_speed4=c.TURN_SPEED, reverse=None, time_count=0, strict=False):
    correction_motor_speed3=motor_speed3
    correction_motor_speed4=motor_speed4
    if robot.front_left_sensor == 0 and robot.front_right_sensor == 0 and robot.mode=="dropping_off":
        pass
    elif robot.front_left_sensor == 0 and robot.front_right_sensor == 0 and strict:
        print("return -10")
        robot.forward(motor3,motor4,100,100)
        return -10
    elif robot.front_left_sensor == 0 and robot.front_right_sensor == 1:
        # If left front sensor does not detect and right does
        robot.queue.append(robot.mode)
        robot.mode="correction"
        robot.right(motor3,motor4,correction_motor_speed3, correction_motor_speed4-40)
        now = utime.ticks_diff(utime.ticks_ms(), robot.start)
        time_count=time_count-now
        #Then turn right
        
    elif robot.front_left_sensor == 1 and robot.front_right_sensor == 0:
        # If left front sensor does not detect and right does
        robot.queue.append(robot.mode)
        robot.mode="correction"
        robot.left(motor3,motor4,correction_motor_speed3-40, correction_motor_speed4)
        now = utime.ticks_diff(utime.ticks_ms(), robot.start)
        time_count=time_count-now

        #Then turn left
        
    return time_count
        
def update_sensors(variables):

    sensors = variables[18]
    robot = variables[16]

    variables[16].left_sensor_previous=robot.left_sensor
    variables[16].front_left_sensor_previous=robot.front_left_sensor
    variables[16].front_right_sensor_previous=robot.front_right_sensor
    variables[16].right_sensor_previous=robot.right_sensor
    
    bits = sensors.read()
    variables[16].left_sensor = 1 if bits & OUTER_LEFT else 0
    variables[16].front_left_sensor = 1 if bits & INNER_LEFT else 0
    variables[16].front_right_sensor = 1 if bits & INNER_RIGHT else 0
    variables[16].right_sensor = 1 if bits & OUTER_RIGHT else 0
    
    return variables


def wait_for_start(button):
    """Block until the start/stop button is pressed.

    The button is the crash sensor named in config.START_STOP_SENSOR (default
    "left" = Crash Sensor L, physical pin 12). Because the Pico runs main.py
    by itself at power-up, waiting here stops the robot from driving off as
    soon as it is switched on. After the press it waits config.START_DELAY_MS
    so the hand that pressed the button is clear of the robot."""
    print("press the start/stop button (%s crash sensor) to start" % config.START_STOP_SENSOR)
    while not button.pressed():
        utime.sleep_ms(5)
    print("starting in 4000 ms")


def button_check(variables):
    '''handles starting/stopping of robot due to button presses, transferred over from main while loop'''
    
    at_start, button, count, dropping_off, drop_off_turn_count,\
    holdoff_until, is_started, left_old_line, map, motor3,\
    motor4, movetime, moving, picking_up, readings,\
    reversing, robot, scantime, sensors, servo1,\
    servo2, skip_forward, tasks, time_count, traffic,\
    turning, using_time , mean_scan, holdoff_until_reverse, leds,\
    yellow_flash_timer, strict, correction_timeout = variables
    #retrieve variables            

    
    
    
    pressed = button.pressed()
    if pressed:

        if is_started:


            # Remember what both motors were doing, so a pause can resume.
            saved_motors = [(m.mDir.value(), m.pwm.duty_u16()) for m in (motor3, motor4)]
            robot.stop(motor3, motor4)
            pause_start = utime.ticks_ms()
            long_press = False
            while button.held():
                button.pressed()          # keeps the button's debouncer up to date
                if utime.ticks_diff(utime.ticks_ms(), pause_start) >= c.LONG_PRESS_MS:
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
            pass
        
        else:
            is_started=True
            print("Started")
            pressed = False          # this press only started the robot
    
    variables = (at_start,              #0
                 button,                #1
                 count,                 #2
                 dropping_off,          #3
                 drop_off_turn_count,   #4
                 holdoff_until,         #5
                 is_started,            #6
                 left_old_line,         #7
                 map,                   #8
                 motor3,                #9
                 motor4,                #10
                 movetime,              #11
                 moving,                #12
                 picking_up,            #13
                 readings,              #14
                 reversing,             #15
                 robot,                 #16
                 scantime,              #17
                 sensors,               #18
                 servo1,                #19
                 servo2,                #20
                 skip_forward,          #21
                 tasks,                 #22
                 time_count,            #23
                 traffic,               #24
                 turning,               #25
                 using_time,            #26
                 mean_scan,             #27
                 holdoff_until_reverse, #28
                 leds,                  #29
                 yellow_flash_timer,    #30
                 strict,                #31
                 correction_timeout,    #32
                 )
    #condense and return new values of variables

    return variables
            


def robot_mode_check(variables, now):
    '''checks the current robot mode and takes action accordingly, transferred over from main while loop'''

    at_start, button, count, dropping_off, drop_off_turn_count,\
    holdoff_until, is_started, left_old_line, map, motor3,\
    motor4, movetime, moving, picking_up, readings,\
    reversing, robot, scantime, sensors, servo1,\
    servo2, skip_forward, tasks, time_count, traffic,\
    turning, using_time , mean_scan, holdoff_until_reverse, leds,\
    yellow_flash_timer, strict, correction_timeout = variables
    #retrieve variables            

    count+=1 #moved from main so still updated one per loop but is now updated when variables is editable

    if now>yellow_flash_timer and (
        motor3.return_state() != 0 or 
        motor4.return_state() != 0 or
        robot.mode == "pick_up" or
        robot.mode == "drop_off"
    ):
        yellow_flash_timer = now + 250
        if leds._pins.get("yellow").value()==1:
            leds.show(yellow=False)
        else:
            leds.show(yellow=True)
    
    if robot.mode == "":
        next_item=robot.queue.pop()
                
        if isinstance(next_item, list):
            # If turning etc
            robot.mode=next_item[0]
            robot.mode_specific=next_item[1:]
                    
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
                        
            if ("DROP" in next_item and not next_item.startswith("JUNCTION")
                and robot.position.startswith("JUNCTION")):
                # Remove straight
                robot.queue = robot.queue[:-2]+[robot.queue[-1]]
                print(robot.queue)

    
            # Stops errors with turning because line spacing is too small
            # If next is a junction and currently at a junction and came from a drop, then add a small reverse
            if (bool(re.match(r"^JUNCTION.*DROP",next_item)) 
                and (bool(re.match(r"^JUNCTION.*DROP",robot.position))) 
                and ("DROP" in robot.previous_position and not robot.previous_position.startswith("JUNCTION"))):
                # Remove straight
                robot.queue = robot.queue[:-1]+["reverse_straight"]+[robot.queue[-1]]
                print(robot.queue)
    
            
            if (robot.position, next_item) in config.TRAFFIC_LIGHT_EDGES:
                robot.queue = robot.queue[:-1] + ["wait_light", robot.queue[-1]]
            
            # When going up and down ramp, only stop at T junctions
            if {robot.position, next_item} == {"TOP_CENTRE", "JUNCTION_TOP_DROP_1"}:
                print("I'm being strict!")
                strict=True

            turn_item = robot.queue[-1] if robot.queue else None
            if (isinstance(turn_item, list) and turn_item[0] == "turning"
                    and turn_item[1] in ("left", "right")):
                corner_turn = (robot.previous_position, robot.position) in c.PIVOT_TURN_AT
                next_slot_turn = (robot.has_box
                                  and bool(re.match(r"^JUNCTION.*BOTTOM.*DROP", next_item))
                                  and bool(re.match(r"^JUNCTION.*BOTTOM.*DROP", robot.position))
                                  and "DROP" in robot.previous_position
                                  and not robot.previous_position.startswith("JUNCTION"))
                if corner_turn or next_slot_turn and robot.has_box:
                    robot.queue[-1] = turn_item + ["pivot"]
                    print("pivot turn " + turn_item[1])

        
    
        else:
            robot.mode=next_item
            if next_item == "collect":
                # Plan collection point
                # Find collection point route and enter. include "pick_up"     
                for key in robot.order_colours(config.PICKUP_ORDER):
                    if map[key]["type"]=="DEAD_END":
                        robot.drop_off_info[0] = 0 if key.startswith("GREEN_") else 1
                        robot.final_objective=key
                        break
                else:
                    # All boxes collected: mark them free again.
                    for key in config.PICKUP_ORDER:
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
                    if robot.drop_off_info[1]==1 and drop_number <= 1:
                        continue
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
                robot.queue.append("TOP_CENTRE")
                robot.queue.append("stop")
                robot.find_route(robot.final_objective)
                robot.queue+=robot.find_route(robot.final_objective)
                robot.mode=""
                            
            elif next_item == "straight":
                
                if strict:
                    robot.forward(motor3,motor4,100,100)
                else:
                    robot.forward(motor3,motor4)
            elif next_item == "reverse_straight":
                print(robot.mode)
                if len(robot.mode_specific) > 2:
                    pivot = len(robot.mode_specific) > 2 and robot.mode_specific[2] == "pivot"
                else:
                    pivot=False
                if pivot:
                    holdoff_until_reverse = now+1000
                else:
                    holdoff_until_reverse = now+400
                reversing=True
                robot.forward(motor3,motor4, motor3.backward_speed-10, motor4.backward_speed-10, reverse=reversing)
                robot.mode="straight"
            elif next_item == "wait_light":
                robot.stop(motor3, motor4)
                tasks.light_reset()
                print("waiting for the traffic light")
                
    elif robot.mode == "straight":
    
                    # Detection
        if  ( 
                ( 
                    ( 
                        (robot.left_sensor == 1 and robot.left_sensor_previous == 0) \
                        or (robot.right_sensor == 1 and robot.right_sensor_previous == 0) 
                    ) 
                    and (
                        not reversing or now >= holdoff_until_reverse 
                    ) 
                    and now>=holdoff_until 
                    or strict
                ) 
                and (
                    (
                        robot.left_sensor == 1 
                        and robot.right_sensor == 1 
                    )
                    or not strict
                )
            ):
            # A junction has been detected
            if strict:
                print("strict is off")
                strict=False
            print("JUNCTION DETECTED")
            robot.previous_position=robot.position
            print(robot.position)
    
            robot.position=robot.objective
                        
                        # In case it wants to reverse
            if reversing:
                utime.sleep_ms(100)
                reversing=False
            robot.mode="" # Moves onto next instruction (which is turning)
            holdoff_until = now + c.JUNCTION_HOLDOFF_MS
            holdoff_until_reverse = now
                    # If not moving straight
        elif motor3.return_state() == 0 or motor4.return_state()==0:
            robot.forward(motor3,motor4, reverse=reversing)
            if strict:
                robot.forward(motor3,motor4,100,100)
            print("motor not forward")
            
        if not reversing: # or not time_count<0
            if now>=correction_timeout:
                if strict:
                    time_count = corrections(robot,motor3,motor4,100,100, reverse=reversing)
                else:
                    time_count = corrections(robot,motor3,motor4, reverse=reversing)
            
                
                    
                
                
    elif robot.mode == "correction":
        
        
        if (((robot.left_sensor == 1 and robot.left_sensor_previous == 0)
            or (robot.right_sensor == 1 and robot.right_sensor_previous == 0)) 
            and (not reversing or now >= holdoff_until_reverse) and now>=holdoff_until 
            and robot.queue[-1]=="straight"
            and not strict):
            # A junction has been detected
                    
            print("JUNCTION DETECTED")
            robot.previous_position=robot.position
            print(robot.position)
    
            robot.position=robot.objective
                        
                        # In case it wants to reverse
            if reversing:
                utime.sleep_ms(200)
                reversing=False 
            if robot.queue[-1]=="straight":
                robot.queue.pop()
            robot.mode="" # Moves onto next instruction (which is turning)
            holdoff_until = now + c.JUNCTION_HOLDOFF_MS
            holdoff_until_reverse = now
            correction_timeout = now + c.CORRECTION_HOLDOFF_MS
        
        
        elif robot.front_left_sensor == 1 and robot.front_right_sensor == 1:
            robot.stop(motor3,motor4)
            robot.mode="" # Moves back to straight or turning
            now = utime.ticks_diff(utime.ticks_ms(), robot.start)
            time_count=now+time_count
            
        # Going up the ramp, if both sensors don't exist, just keep going forwards
        elif robot.front_left_sensor == 0 and robot.front_right_sensor == 0 and strict:
            robot.forward(motor3, motor4, motor3.forward_speed, motor4.forward_speed)
            robot.mode="" # Moves back to straight or turning
            now = utime.ticks_diff(utime.ticks_ms(), robot.start)
            time_count=now+time_count
        
        # Detect if there is a dead_end. If there is, then stop going straight.
        elif robot.front_left_sensor == 0 and robot.front_right_sensor == 0 and robot.queue[-1]=="drop_off":
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
                
        if not turning:
                        # Turning hasn't started, do the turns
            turning=True
            robot.mode_extra_specific="move_straight"
            
                        
                        
            if skip_forward:
                forward_ms = 0
            else:
                forward_ms = c.NODE_FORWARD_MS
            pivot = len(robot.mode_specific) > 2 and robot.mode_specific[2] == "pivot"
            if pivot and forward_ms > 0:
                forward_ms = c.PIVOT_FORWARD_MS
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
                time_count=now+int(forward_ms/8)
                        
    
        elif robot.mode_extra_specific=="move_straight":
            # Make sure does not change off line
            if now>=time_count:
                
                robot.mode_extra_specific="move_turning"
                left_old_line=False
                turn_min = c.TURN_MIN_MS
                            
                            
                if robot.mode_specific[0]=="straight":
                    # MAYBE check if will trigger junction (need to implement)
                    robot.forward(motor3,motor4,reverse=reversing)
                    robot.mode=""
                    turning=False
                                # Move back to straight movement
                                # initial turn
                elif robot.mode_specific[0]=="left":
                    if len(robot.mode_specific) > 2 and robot.mode_specific[2] == "pivot":
                        robot.pivot(motor3, motor4, "left", c.TURN_SPEED)
                        time_count=now+c.PIVOT_TURN_MIN_MS
                    else:
                        robot.spin(motor3, motor4, "left", c.TURN_SPEED)
                        time_count=now+turn_min
                elif robot.mode_specific[0]=="right":
                    if len(robot.mode_specific) > 2 and robot.mode_specific[2] == "pivot":
                        robot.pivot(motor3, motor4, "right", c.TURN_SPEED)
                        time_count=now+c.PIVOT_TURN_MIN_MS
                    else:
                        robot.spin(motor3, motor4, "right", c.TURN_SPEED)
                        time_count=now+turn_min
                elif robot.mode_specific[0]=="about":
                    about_direction = "right"
                    if len(robot.mode_specific) > 2:
                        about_direction = robot.mode_specific[2]
                    robot.about(motor3,motor4, c.TURN_SPEED, direction=about_direction)
                    time_count=now+turn_min*1.90
                at_start=False


        elif robot.mode_extra_specific=="move_turning":
                        # If we need crash detection or anything here then do it

            if now>time_count:
                if robot.front_left_sensor == 1 and robot.front_right_sensor == 1:
                    robot.stop(motor3,motor4)
                    reverse_ms = c.AFTER_TURN_REVERSE_MS
                    if robot.queue and robot.queue[-1] == "pick_up":
                        reverse_ms = c.PICKUP_TURN_REVERSE_MS
                    if (robot.mode_specific[0] in ("left", "right")
                            and reverse_ms > 0):
                        robot.forward(motor3, motor4, reverse=True)
                        utime.sleep_ms(reverse_ms)
                        robot.stop(motor3, motor4)
                    robot.mode="" # Moves to next step
                    robot.direction=turn_to_direction(robot.direction, robot.mode_specific[0])
                    print(robot.direction)
    
                    turning=False
                    holdoff_until = 0
                                
                                
                                
    elif robot.mode == "pick_up":
        if not picking_up:
            picking_up=True
            robot.previous_position=robot.position
            robot.position=robot.objective          # in the box (e.g. GREEN_1)
            # the U-turn spins away from the neighbouring box (config)
            print("now")
            gripper_carry(variables[19], variables[20])
            robot.pickup = PickUp(
                robot, 
                motor3, 
                motor4, 
                servo1, 
                servo2,
                tasks,
                leds,
                config.PICKUP_UTURN_DIRECTION.get(robot.final_objective, "left")
            )
            
            
            robot.pickup.start(now)
        elif robot.pickup.update(now):
            
            if robot.pickup.ok:
                # Back on the main line, at the box's junction, facing the
                # way it came in reversed (the U-turn). The next route
                # starts from this junction; the next turn first drives
                # NODE_FORWARD_MS so the wheels get onto the junction, exactly
                # as after any junction found while driving straight.
                
                
                
                
                map[robot.final_objective]["type"]="COLLECTED"
                if robot.pickup.no_box:
                    robot.queue.append("collect")
                    print("GOING FOR ANOTHER (space was empty)")
                else:
                    robot.has_box=True
                    level=robot.pickup.level
                    confidence=robot.pickup.confidence
                    
                    robot.drop_off_info[1] = 1 if level == "N" else 0
                robot.previous_position=robot.position
                robot.position="JUNCTION_" + robot.final_objective
                robot.mode=""
                picking_up=False
                if config.STOP_AFTER_PICKUP:
                    robot.stop(motor3, motor4)
                    is_started=False
                    print("STOP_AFTER_PICKUP: box magnet %s -> %s level; stopped on the main line"
                          " - press the button to go on" % (robot.pickup.pole, level))
            else:
                # Something went wrong (no cross line, no box, ...): the
                # sequence has stopped the motors. Stop the run here so a
                # person can look; a press of the button goes on.
                print("PICK-UP FAILED: %s - stopped, press the button to go on" % robot.pickup.message)
                is_started=False
                robot.mode=""
    elif robot.mode == "drop_off":
        
        
        if not dropping_off:
                        #Do some actual grabbing stuff etc
            
            robot.previous_position=robot.position
            robot.position=robot.objective
            
            print(robot.position)
            print("DROPPING STUFF")
                        # Turning hasn't started, do the turns
            dropping_off=True
            robot.mode_extra_specific="move_straight_and_detect"
                        
                        
            robot.forward(motor3,motor4)
                        # If at top, then move very slightly less
            if robot.drop_off_info[1]==0:
                if robot.drop_off_info[0]==0:
                    drop_off_distance=c.DISTANCE_TO_DROP_OFF_SCAN
                else:
                    drop_off_distance=c.DISTANCE_TO_DROP_OFF_SCAN-c.MINUS_DISTANCE_TO_DROP_OFF_SCAN_RIGHT
            else:
                drop_off_distance=c.DISTANCE_TO_DROP_OFF_SCAN-c.MINUS_DISTANCE_FOR_TOP_DROP_OFF
            drop_off_distance+=c.AFTER_TURN_REVERSE_MS

            time_count=now+drop_off_distance
        elif robot.mode_extra_specific=="move_straight_and_detect":
            
            if now>=time_count:
                robot.stop(motor3,motor4)
                robot.mode_extra_specific="move_turning"
                drop_off_turn_count=0
                print("moving_turning")
            
            elif motor3.return_state() == 0 or motor4.return_state()==0:
                robot.forward(motor3,motor4, reverse=reversing)
            elif now<=time_count-200:
                time_count = corrections(robot, motor3, motor4, time_count=time_count)
            
        elif robot.mode_extra_specific=="move_turning":
                        # If this is coming from drop_off point
            if drop_off_turn_count==0:
                skip_forward=True
                if robot.drop_off_info[0]==1:
                    robot.queue+=["drop_off", ["turning", "about", 90, "left"]]
                else:
                    robot.queue+=["drop_off", ["turning", "about", 90, "left"]]
                robot.mode=""
            elif drop_off_turn_count==1:
                utime.sleep_ms(100)
                for i in range(5):
                    d = tasks.rear_distance()
                    if d or robot.drop_off_info[1]==1:
                        break
                    else:
                        utime.sleep_ms(300)
                        print("trying again, reading was: %s" %tasks.last_rear_readings)
                print("rear sensor readings: %s" % tasks.last_rear_readings)
                # No reading = nothing in range behind the robot = empty slot
                # (1000 = empty).
                mean_scan = d if d is not None else 1000
                
                
                
                print("DISTANCE MEASURED:"+str(mean_scan))
                if mean_scan>c.DISTANCE_TO_SCAN_DROP_OFF:
                    # Use sensor to detect if box is there or not
                    print("NO BOX SEEN")
                    skip_forward=True
                    if robot.drop_off_info[0]==1:
                        robot.queue+=["drop_off", ["turning", "about", 90, "left"]]
                    else:
                        robot.queue+=["drop_off", ["turning", "about", 90, "left"]]

                    robot.mode=""
                else:
                    print("BOX THERE ALREADY")
                    # If there already is a box
                    map[robot.final_objective]["type"]="FILLED"
                    robot.queue+=["deliver"]
                    if robot.drop_off_info[1]==1:
                        
                        robot.forward(motor3,motor4, 60, 60, reverse=True)
                        utime.sleep_ms(400)
                        print("reverse a bit since at top")
                    robot.mode=""
                    dropping_off=False
            elif drop_off_turn_count==2:
                gripper_lift(servo1, servo2)
                if c.DROP_RUNUP_MS > 0:
                    robot.forward(motor3, motor4, reverse=True)
                    utime.sleep_ms(c.DROP_RUNUP_MS)
                    robot.stop(motor3, motor4)
                    utime.sleep_ms(500)
                now = utime.ticks_diff(utime.ticks_ms(), robot.start)
                robot.mode_extra_specific="move_and_drop_off"
                
                robot.forward(motor3,motor4)
                time_count=now+c.DISTANCE_TO_DROP_POINT+c.DROP_RUNUP_MS
        elif robot.mode_extra_specific=="move_and_drop_off":
            if (now>=time_count and robot.drop_off_info[1] == 0) or (robot.drop_off_info[1] == 1 and (robot.front_left_sensor == 0 and robot.front_right_sensor == 0)):
                robot.stop(motor3, motor4)
                
                # Do the servo motions and moving, using sleep as well for moving forwards as 
                            # this needs to be hugely precise
                            # Drop off
                if robot.drop_off_info[1]==0:
                    robot.forward(motor3,motor4, c.DRIVE_SPEED/2, c.DRIVE_SPEED/2)
                    utime.sleep_ms(100)
                robot.stop(motor3,motor4)
                utime.sleep_ms(400)
                
                release_box(servo2)
                
                robot.forward(motor3,motor4, 70, 70, reverse=True)
                utime.sleep_ms(700)
                
                robot.stop(motor3,motor4)
                
                
                
                lowest = servo2.low - 10                
                servo2.move_slowly(lowest, config.GRIPPER_STEP_MS, low=lowest)
                
                
                robot.forward(motor3,motor4, c.DRIVE_SPEED/2, c.DRIVE_SPEED/2)
                utime.sleep_ms(1300)
                
                
                
                robot.forward(motor3,motor4, 70, 70, reverse=True)
                utime.sleep_ms(400)
            
            
                gripper_safe(servo1, servo2)
                
                
                robot.box_count+=1

                print("EVERYTHING IS DROPPED OFF. COUNT: "+str(robot.box_count))
                robot.has_box=False
                dropping_off=False
                            
                map[robot.final_objective]["type"]="FILLED"
                            
                robot.mode=""
                robot.previous_position=robot.position
                robot.position=robot.position.replace("JUNCTION_", "")
                print("I AM AT: " +str(robot.position))
                print("MY DIRECTION IS: "+ str(robot.direction))
                
                # No more box so reset red and green
                leds.show(green=False)                        
                leds.show(red=False)
                
                
                
            else:
                if motor3.return_state() == 0 or motor4.return_state() == 0:
                    robot.forward(motor3,motor4, reverse=reversing)
                if now<time_count-500:
                    time_count = corrections(robot, motor3, motor4, time_count=time_count)
            
            
    
                    
    elif robot.mode == "wait_light":
        if tasks.light_update() == "green":
            print("traffic light green - go (waited %d ms)" % tasks.light_waited_ms())
            robot.mode=""
    
    elif robot.mode == "stop":
        is_started=False
        robot.stop(motor3, motor4)
        print("REACHED START")
    
    variables = (at_start,              #0
                 button,                #1
                 count,                 #2
                 dropping_off,          #3
                 drop_off_turn_count,   #4
                 holdoff_until,         #5
                 is_started,            #6
                 left_old_line,         #7
                 map,                   #8
                 motor3,                #9
                 motor4,                #10
                 movetime,              #11
                 moving,                #12
                 picking_up,            #13
                 readings,              #14
                 reversing,             #15
                 robot,                 #16
                 scantime,              #17
                 sensors,               #18
                 servo1,                #19
                 servo2,                #20
                 skip_forward,          #21
                 tasks,                 #22
                 time_count,            #23
                 traffic,               #24
                 turning,               #25
                 using_time,            #26
                 mean_scan,             #27
                 holdoff_until_reverse, #28
                 leds,                  #29
                 yellow_flash_timer,    #30
                 strict,                #31
                 correction_timeout,    #32
                 )
    #condense and return new values of variables

    return variables, now


def initialise_main_variables():
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
    mean_scan=100000
    holdoff_until_reverse=0
    yellow_flash_timer=0
    strict=False
    correction_timeout=0
    
    
    left_old_line=False   # turning: True once both centre sensors have left the old line
    holdoff_until=0       # ignore junctions until now reaches this (see JUNCTION_HOLDOFF_MS)
    skip_forward=False

    robot=Robot()
    print(robot.position)
    reset_map()
    map=MAP.MAP.copy()
    # Measure elapsed time
    robot.start = utime.ticks_ms()

    sensors=line_sensor.LineSensorArray([20,21,19,18],True)

    tasks = TaskSensors.from_config()
    
    leds = StatusLeds.from_config()

    print("magnet sensor zero point: %s mV" % tasks.calibrate())


    button = StartStopButton.from_config()


    readings=[]

    drop_off_turn_count=100
    
    motor3 = motor.Motor(dirPin=4, PWMPin=5)  # Motor 3 is controlled from Motor Driv2 #1, which is on GP4/5
    motor4 = motor.Motor(dirPin=7, PWMPin=6)  # Motor 4 is controlled from Motor Driv2 #1, which is on GP4/5
    
    servo1 = Servo(config.SERVO_PINS["servo1"])
    servo2 = Servo(config.SERVO_PINS["servo2"])

    motor3.forward_speed = c.DRIVE_SPEED
    motor3.backward_speed = c.DRIVE_SPEED
    motor4.forward_speed = c.DRIVE_SPEED
    motor4.backward_speed = c.DRIVE_SPEED

    is_started=True

    variables = (at_start,              #0
                 button,                #1
                 count,                 #2
                 dropping_off,          #3
                 drop_off_turn_count,   #4
                 holdoff_until,         #5
                 is_started,            #6
                 left_old_line,         #7
                 map,                   #8
                 motor3,                #9
                 motor4,                #10
                 movetime,              #11
                 moving,                #12
                 picking_up,            #13
                 readings,              #14
                 reversing,             #15
                 robot,                 #16
                 scantime,              #17
                 sensors,               #18
                 servo1,                #19
                 servo2,                #20
                 skip_forward,          #21
                 tasks,                 #22
                 time_count,            #23
                 traffic,               #24
                 turning,               #25
                 using_time,            #26
                 mean_scan,             #27
                 holdoff_until_reverse, #28
                 leds,                  #29
                 yellow_flash_timer,    #30
                 strict,                #31
                 correction_timeout,    #32
                 )
    return variables