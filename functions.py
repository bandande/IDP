import constants as c
from collections import deque
from motor import Motor
from sensors.line_processing import OUTER_LEFT, INNER_LEFT, INNER_RIGHT, OUTER_RIGHT
import utime
import config
from control.task_sensors import TaskSensors
import re
import motor
from actuators.servo import Servo
import MAP
import sensors.line_sensor as line_sensor
from control.start_stop import StartStopButton
# ADDED (2026-10-27): the pick-up sequence (red / green box) and the
# gripper helpers now live in their own modules, so the bench calibration
# test (tests/test_pickup.py) runs exactly the same code as the robot.
from control.pickup import PickUp
from control.dropoff import release_and_push     # ADDED (2026-10-27): let go + push the box in
from control.dropoff import drive_in_and_drop    # ADDED (2026-10-27): drive into the branch, then the above
from actuators.gripper import gripper_end, gripper_carry, gripper_start, grab_box, release_box


def degrees_to_time(degrees):
    count=0
    count=c.full_turn_time*degrees/360
    return count

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
    # FIX (2026-10-27): reset the map in RAM only. The old version rewrote
    # MAP.py on the flash at EVERY start. If the power dipped or the program
    # was interrupted during that write, MAP.py was left broken and the next
    # start crashed at "from MAP import MAP" (OSError: 84 on the Pico on
    # 2026-10-23). The robot code never writes the map back to the file
    # anyway - it only changes the node types in memory while it runs - so
    # setting them back in memory does the same job without touching flash.
    for node in MAP.MAP.values():
        if node.get("type") in old_phrases:
            node["type"] = "DEAD_END"


    

class Robot:
    
    def reset(self):
        # self.position = "JUNCTION_LEFT_BOTTOM"
        # self.direction = "west"
        

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
        # specific_mode=()
        
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
        
        self.drop_off_info=[0,0,0]
        
        self.start=0
    
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
                motor3.off()
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
                motor4.off()
    def about(self, motor3, motor4, motor3_speed=None, motor4_speed=None):
            """motor3 is left. motor4 is right
            """
            if not motor3_speed:
                motor3_speed=motor3.forward_speed
            if not motor4_speed:
                motor4_speed=motor4.backward_speed
            self.motor_left=True
            self.motor_right=True
            motor3.Forward(motor3_speed)
            motor4.Reverse(motor4_speed)
    def backward(self, motor3, motor4, motor3_speed=None, motor4_speed=None):
            """motor3 is left. motor4 is right
            """
            if not motor3_speed:
                motor3_speed=motor3.backward_speed
            if not motor4_speed:
                motor4_speed=motor4.backward_speed
            self.motor_left=True
            self.motor_right=True
            motor3.Reverse(motor3_speed)
            motor4.Reverse(motor4_speed)
    def left_backward(self, motor3, motor4, motor3_speed=None, motor4_speed=None):
            """motor3 is left. motor4 is right
            """
            if not motor3_speed:
                motor3_speed=motor3.backward_speed
            if not motor4_speed:
                motor4_speed=motor4.backward_speed
            self.motor_left=True
            self.motor_right=False
            motor3.Reverse(motor3_speed)
            motor4.off()
    def right_backward(self, motor3, motor4, motor3_speed=None, motor4_speed=None):
            """motor3 is left. motor4 is right
            """
            if not motor3_speed:
                motor3_speed=motor3.backward_speed
            if not motor4_speed:
                motor4_speed=motor4.backward_speed
            self.motor_left=False
            self.motor_right=True
            motor3.off()
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
        """ADDED (2026-10-15): turn on the spot, "left" or "right".

        One wheel drives forwards and the other backwards at the same power, so
        the robot rotates about the middle of its wheel axle and stays on the
        junction. Robot.left / Robot.right are NOT used for junction turns: they
        stop one wheel and pivot about it, which moves the robot sideways off
        the junction while it turns, so it ends up beside the new line.
        motor3 is the left wheel, motor4 the right wheel.
        """
        if direction == "left":
            motor3.Reverse(speed)
            motor4.Forward(speed)
        else:
            motor3.Forward(speed)
            motor4.Reverse(speed)
        self.motor_left = True
        self.motor_right = True

    
    
        
    
    def find_route(self, location):
        # Check that both locations exist
        if self.position not in MAP.MAP or location not in MAP.MAP:
            return ["Error position and location not found"]

        # Queue of locations to explore
        # FIX (2026-10-15): MicroPython's collections.deque takes the maximum
        # length as a required second argument and must start empty, so the
        # old `deque([self.position])` raised "TypeError: function missing 1
        # required positional arguments" on the Pico. len(MAP) is always
        # enough room: in this breadth-first search every location is added
        # to the queue at most once. (Works the same in CPython.)
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
            # FIX (2026-10-15): dict views (.keys(), .values()) cannot be
            # indexed or searched with .index() - neither in CPython nor in
            # MicroPython - so the old one-liner
            #   MAP[self.position].keys()[MAP[self.position].values().index(location)]
            # always raised an error. Instead, look through the four
            # directions for the one whose neighbour is `location`.
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
    
    
    def identify_junction(self):
        # This can be called to see which kinds of junction it can be
        pass
    
def corrections(robot, motor3, motor4, motor_speed3=c.TURN_SPEED, motor_speed4=c.TURN_SPEED, reverse=None, time_count=0):
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




# CHANGED (2026-10-27): gripper_end, gripper_carry, gripper_start, grab_box
# and release_box moved to actuators/gripper.py (imported at the top).


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


def button_check(variables):
    '''handles starting/stopping of robot due to button presses, transferred over from main while loop'''
    
    at_start, button, count, dropping_off, drop_off_turn_count, holdoff_until, is_started, left_old_line, map, motor3, motor4, movetime, moving, picking_up, readings, reversing, robot, scantime, sensors, servo1, servo2, skip_forward, tasks, time_count, traffic, turning, using_time , mean_scan= variables
    #retrieve variables            

    
    
    
    pressed = button.pressed()
    if pressed:

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

            # FIX (2026-10-20): read the button ONCE per loop. button.pressed()
            # returns True only once for each press. The old code called it twice:
            #     if button.pressed() and not is_started:    (1)
            #     ...
            #         if button.pressed():                   (2)
            # While the robot was running, call (1) used up the press and then
            # threw it away (because is_started was True), so call (2) almost
            # never saw it: that is why the button sometimes did not stop the
            # robot.


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
                 )
    #condense and return new values of variables

    return variables
            

def robot_mode_check(variables, now):
    '''checks the current robot mode and takes action accordingly, transferred over from main while loop'''

    at_start, button, count, dropping_off, drop_off_turn_count, holdoff_until, is_started, left_old_line, map, motor3, motor4, movetime, moving, picking_up, readings, reversing, robot, scantime, sensors, servo1, servo2, skip_forward, tasks, time_count, traffic, turning, using_time, mean_scan = variables
    #retrieve variables            

    count+=1 #moved from main so still updated one per loop but is now updated when variables is editable

    if robot.mode == "":
        next_item=robot.queue.pop()
                
        if isinstance(next_item, list):
            # If turning etc
            robot.mode=next_item[0]
            robot.mode_specific=next_item[1:]
            # f.degrees_to_time(next_item[2]) # gets the count for that specific turn
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
                        
            # ADDED (2026-10-27): Prevent driving on into the drop branch.
            # Like the pick-up above: from a drop JUNCTION into its DROP slot
            # (a dead end) only the turn is kept; the "drop_off" mode then
            # starts right after the turn and drives in itself for
            # config.DROP_DRIVE_MS (measured from the turn, see config.py).
            if ("DROP" in next_item and not next_item.startswith("JUNCTION")
                and robot.position.startswith("JUNCTION")):
                # Remove straight
                robot.queue = robot.queue[:-2]+[robot.queue[-1]]
                print(robot.queue)

            # Prevent moving forwards when picking up the box.
            if (bool(re.match(r"^JUNCTION.*DROP",next_item))
                and ("DROP" in robot.position and not robot.position.startswith("JUNCTION"))):
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
                # CHANGED (2026-10-27): look at the boxes in a FIXED order
                # (config.PICKUP_ORDER). The old loop went through map.keys(),
                # whose order MicroPython does not keep, so the box chosen was
                # effectively random (on the robot it went to GREEN_2 first).
                # GREEN boxes are delivered on the LEFT side (drop_off_info[0]
                # = 0), RED boxes on the RIGHT side (1), as before.
                for key in config.PICKUP_ORDER:
                    if map[key]["type"]=="DEAD_END":
                        robot.drop_off_info[0] = 0 if key.startswith("GREEN_") else 1
                        robot.final_objective=key
                        break
                else:
                    # All boxes collected: mark them free again, as before.
                    # FIX (2026-10-27): the old line used
                    # key.startswith(("GREEN_","RED_")) - a tuple argument,
                    # which MicroPython's str.startswith does not accept (it
                    # would have crashed here once every box was taken).
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
            holdoff_until = now + c.JUNCTION_HOLDOFF_MS   # ADDED (2026-10-20)
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
                forward_ms = c.NODE_FORWARD_MS
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
                turn_min = c.TURN_MIN_MS
                            
                            
                if robot.mode_specific[0]=="straight":
                    # MAYBE check if will trigger junction (need to implement)
                    robot.forward(motor3,motor4,reverse=reversing)
                    robot.mode=""
                    turning=False
                                # Move back to straight movement
                                # initial turn
                elif robot.mode_specific[0]=="left":
                    robot.spin(motor3, motor4, "left", c.TURN_SPEED)
                    time_count=now+turn_min
                elif robot.mode_specific[0]=="right":
                    robot.spin(motor3, motor4, "right", c.TURN_SPEED)
                    time_count=now+turn_min
                elif robot.mode_specific[0]=="about":
                    robot.about(motor3,motor4)
                    time_count=now+turn_min*2
                at_start=False
    
    
        elif robot.mode_extra_specific=="move_turning":
                        # If we need crash detection or anything here then do it

            if now<time_count:
                pass
            else:
                if robot.front_left_sensor == 1 and robot.front_right_sensor == 1:
                    robot.stop(motor3,motor4)
                    robot.mode="" # Moves to next step
                    robot.direction=turn_to_direction(robot.direction, robot.mode_specific[0])
                    print(robot.direction)
    
                    turning=False
                                # CHANGED (2026-10-20): no junction hold-off after a
                                # turn (see JUNCTION_HOLDOFF_MS at the top).
                    holdoff_until = 0
                                
                                
                                
    elif robot.mode == "pick_up":
        # CHANGED (2026-10-27): the pick-up is now control/pickup.py's PickUp
        # sequence (the partner's scan-and-reverse draft that was here never
        # finished: it set mode_extra_specific to "return turn" but tested for
        # "return_turn", and it grabbed even when it saw no box).
        # It starts right after the robot has turned from the main line into
        # the red / green box: follow the box's line to its white cross line
        # (all four sensors white), straight on until the front distance
        # sensor reads config.GRAB_STOP_MM, grab (servo1 down, servo2 closed,
        # servo1 up), U-turn, follow the line back to the main line and stop
        # there. See control/pickup.py and the PICKUP_... settings in config.py.
        if not picking_up:
            picking_up=True
            robot.previous_position=robot.position
            robot.position=robot.objective          # in the box (e.g. GREEN_1)
            # the U-turn spins away from the neighbouring box (config)
            robot.pickup = PickUp(robot, motor3, motor4, servo1, servo2, tasks,
                                  config.PICKUP_UTURN_DIRECTION.get(robot.final_objective, "left"))
            robot.pickup.start(now)
        elif robot.pickup.update(now):
            picking_up=False
            if robot.pickup.ok:
                # Back on the main line, at the box's junction, facing the
                # way it came in reversed (the U-turn). The next route
                # starts from this junction; the next turn first drives
                # NODE_FORWARD_MS so the wheels get onto the junction, exactly
                # as after any junction found while driving straight.
                # CHANGED (2026-10-27): the box's magnet was already read by
                # the pick-up sequence right after the grab (robot.pickup.pole)
                # - no second reading here.
                level = robot.pickup.level
                robot.drop_off_info[1] = 1 if level == "TOP" else 0
                map[robot.final_objective]["type"]="COLLECTED"
                robot.has_box=True
                robot.previous_position=robot.position
                robot.position="JUNCTION_" + robot.final_objective
                robot.direction=turn_to_direction(robot.direction, "about")
                robot.mode=""
                # ADDED (2026-10-27): for now the robot only REPORTS the pole
                # and stops on the main line after the U-turn (it does not
                # yet turn left / right because of it). A press of the button
                # goes on with the route.
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
                    #Do some actual dropping off stuff etc
        utime.sleep(2)
        print("DROPPING STUFF")
                    # This should have the dropping off and so on code
        # =======================
        robot.previous_position=robot.position
        robot.position=robot.objective
        map[robot.final_objective]["type"]="FILLED"
                                    
        robot.mode=""
        # ADDED (2026-10-26, test build): open servo2 a little to let go of
        # the box, then stop (config.STOP_AFTER_DROP; a press goes on).
        # CHANGED (2026-10-27): after letting go, back off, close the gripper
        # to config.SERVO_PUSH_ANGLE, push the box in, back out and open the
        # gripper again (control/dropoff.py release_and_push).
        # CHANGED (2026-10-27): the drop-off now starts right after the turn
        # into the drop branch (see "Prevent driving on into the drop branch"
        # above): arm to config.DROP_SERVO1_ANGLE, follow the branch's line
        # for config.DROP_DRIVE_MS, then let go and push
        # (control/dropoff.py drive_in_and_drop; values from the calibration).
        robot.stop(motor3, motor4)
        drive_in_and_drop(robot, motor3, motor4, servo1, servo2, line=sensors)   # follows the branch's line
        if config.STOP_AFTER_DROP:
            is_started=False
            print("STOP_AFTER_DROP: stopped after the drop-off - press the button to go on")
        
        # if not dropping_off:
        #                 #Do some actual grabbing stuff etc
        #     robot.previous_position=robot.position
        #     robot.position=robot.objective
        #     print(robot.position)
        #     utime.sleep(1)
        #     print("DROPPING STUFF")
        #                 # Turning hasn't started, do the turns
        #     dropping_off=True
        #     robot.mode_extra_specific="move_straight_and_detect"
                        
        #     robot.forward(motor3,motor4)
        #                 # If at top, then move very slightly less
        #     if robot.drop_off_info[1]==0:
        #         drop_off_distance=c.DISTANCE_TO_DROP_OFF_SCAN
        #     else:
        #         drop_off_distance=c.DISTANCE_TO_DROP_OFF_SCAN-200
                            
        #     time_count=now+drop_off_distance
        # elif robot.mode_extra_specific=="move_straight_and_detect":
        #     if now>=time_count:
        #         robot.stop(motor3,motor4)
        #         robot.mode_extra_specific="move_turning"
        #         drop_off_turn_count=0
                        
        #     corrections(robot, motor3, motor4, time_count=time_count)
        # elif robot.mode_extra_specific=="move_turning":
        #                 # If this is coming from drop_off point
        #     if drop_off_turn_count==0:
        #         skip_forward=True
        #         robot.queue+=["drop_off", ["turning", "about", 90]]
        #         robot.mode=""
        #     elif drop_off_turn_count==1:
        #         for i in range(10):
        #             if tasks.grab_tof.poll():
        #                 d = tasks.grab_tof.get_distance()
        #                 if d is not None:
        #                     readings.append(d)
        #         read=readings[-10:]
        #         read.sort()
        #         mean_scan=read[5]
        #         print("DISTANCE MEASURED:"+str(mean_scan))
        #         if mean_scan>c.DISTANCE_TO_SCAN_DROP_OFF:
        #             # Use sensor to detect if box is there or not
        #             print("NO BOX SEEN")
        #             skip_forward=True
        #             robot.queue+=["drop_off", ["turning", "about", 90]]
        #             robot.mode=""
        #         else:
        #             print("BOX THERE ALREADY")
        #             # If there already is a box
        #             map[robot.final_objective]["type"]="FILLED"
        #             robot.queue+=["deliver"]
        #             robot.mode=""
        #             dropping_off=False
        #     elif drop_off_turn_count==2:
        #         robot.mode_extra_specific="move_and_drop_off"
        #         robot.forward(motor3,motor4)
        #         time_count=now+c.DISTANCE_TO_DROP_POINT
        # elif robot.mode_extra_specific=="move_and_drop_off":
        #     if now>=time_count:
        #         robot.stop(motor3, motor4)
        #         # Do the servo motions and moving, using sleep as well for moving forwards as 
        #                     # this needs to be hugely precise
        #                     # Drop off
        #         robot.forward(motor3,motor4, c.DRIVE_SPEED/2, c.DRIVE_SPEED/2)
        #         utime.sleep_ms(20)
        #         servo1.set_angle(0)
        #         robot.forward(motor3,motor4, c.DRIVE_SPEED/2, c.DRIVE_SPEED/2, reverse=True)
                
        #         servo1.set_angle(0)
        #         servo2.set_angle(0)
                
                
        #         print("EVERYTHING IS DROPPED OFF. COUNT: "+str(robot.box_count))
        #         robot.box_count+=1
        #         robot.has_box=False
                            
        #         map[robot.final_objective]["type"]="FILLED"
                            
        #         robot.mode=""
        #     corrections(robot, motor3, motor4, time_count=time_count)
    
                    
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

    robot=Robot()
    print(robot.position)
    reset_map()
    map=MAP.MAP.copy()
    # Measure elapsed time
    robot.start = utime.ticks_ms()
    # elapsed = utime.ticks_diff(utime.ticks_ms(), start)

    sensors=line_sensor.LineSensorArray([20,21,19,18],True)

    # ADDED (2026-10-20): start the task sensors. calibrate() measures the
    # magnet sensor's no-magnet zero point, so it runs here, in the start box,
    # before the start button is pressed (no box / magnet near the robot).
    tasks = TaskSensors.from_config()

    print("magnet sensor zero point: %s mV" % tasks.calibrate())



    # ADDED (2026-10-15, safety): start / stop button. Nothing moves until
    # the button is pressed (see wait_for_start above).
    button = StartStopButton.from_config()





    readings=[]

    drop_off_turn_count=100
    
    motor3 = motor.Motor(dirPin=4, PWMPin=5)  # Motor 3 is controlled from Motor Driv2 #1, which is on GP4/5
    motor4 = motor.Motor(dirPin=7, PWMPin=6)  # Motor 4 is controlled from Motor Driv2 #1, which is on GP4/5
    
    # FIX (2026-10-26, test build): by name - MicroPython dicts do not keep
    # their order, so .items() could hand servo2 out first.
    servo1 = Servo(config.SERVO_PINS["servo1"])
    servo2 = Servo(config.SERVO_PINS["servo2"])

    # ADDED (2026-10-15): use DRIVE_SPEED (top of this file) instead of the
    # Motor class default of 100 %. Robot.forward / left / right / backward /
    # about use these two values whenever no explicit speed is passed.
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
                 )
    return variables