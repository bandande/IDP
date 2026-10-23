from MAP import MAP
import constants as c
from collections import deque
from motor import Motor
from sensors.line_processing import OUTER_LEFT, INNER_LEFT, INNER_RIGHT, OUTER_RIGHT
import utime
import config
from control.task_sensors import TaskSensors
import re

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

def reset_map(old_phrases=["COLLECTED", "OCCUPIED"]):
    # Read in the file
    with open('MAP.py', 'r') as file:
        filedata = file.read()

    # Replace the target string
    for old_phrase in old_phrases:
        filedata = filedata.replace(old_phrase, "DEAD_END")

    # Write the file out again
    with open('MAP.py', 'w') as file:
        file.write(filedata)
        

    

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
        if self.position not in MAP or location not in MAP:
            return ["Error position and location not found"]

        # Queue of locations to explore
        # FIX (2026-10-15): MicroPython's collections.deque takes the maximum
        # length as a required second argument and must start empty, so the
        # old `deque([self.position])` raised "TypeError: function missing 1
        # required positional arguments" on the Pico. len(MAP) is always
        # enough room: in this breadth-first search every location is added
        # to the queue at most once. (Works the same in CPython.)
        queue = deque((), len(MAP))
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
                neighbour = MAP[current].get(direction)

                if neighbour is None:
                    continue

                if neighbour not in MAP:
                    continue

                if neighbour not in previous:
                    previous[neighbour] = current
                    queue.append(neighbour)

        # No route exists
        return ["Error no route found"]
    
    def move_to(self, location):
        # From one point to adjacent point
        if MAP.get(self.position) and MAP.get(location):
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
                if MAP[self.position].get(direction) == location:
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


def button_check(button, is_started, robot, motor3, motor4):
    '''handles starting/stopping of robot due to button presses, transferred over from main while loop'''
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
    return is_started
            

def robot_mode_check(robot, map, motor3, motor4, tasks, now, holdoff_until, skip_forward, reversing, time_count, turning, 
                     drop_off_turn_count, at_start, picking_up, readings, scantime, movetime, servo1, servo2, dropping_off, left_old_line):
    '''checks the current robot mode and takes action accordingly, transferred over from main while loop'''
    
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
            if robot.front_left_sensor == 0 and robot.front_right_sensor == 0:
                left_old_line = True
            if now<time_count or not left_old_line:
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
        robot.previous_position=robot.position
        robot.position=robot.objective
        utime.sleep(2)
        map[robot.final_objective]["type"]="COLLECTED"
        print("Picking up stuff")
        robot.mode=""
        
                
        # if not picking_up:
        #                 #Do some actual grabbing stuff etc
        #     robot.previous_position=robot.position
        #     robot.position=robot.objective
        #     print(robot.position)
        #     utime.sleep(1)
        #     print("GRABBING STUFF")
        #     # Turning hasn't started, do the turns
        #     picking_up=True
        #     robot.mode_extra_specific="scan"
                        
                        
        #     readings = [10000 for i in range(10)]
        #     time_count=now+c.SCAN_TURN_TIME
        #     scantime=now
        #     robot.spin(motor3, motor4, "left", c.TURN_SPEED)
                        
    
        # elif robot.mode_extra_specific=="scan":
        #     if now>=time_count and motor3.return_state()==-1:
        #         time_count+=c.SCAN_TURN_TIME*2
        #         robot.spin(motor3, motor4, "right", c.TURN_SPEED/2)
        #     elif now>=time_count and motor3.return_state()==1:
        #         time_count+=c.SCAN_TURN_TIME*2
        #         robot.spin(motor3, motor4, "left", c.TURN_SPEED/2)
                        
        #     if now%5==0:
        #         if tasks.front_tof.poll():
        #             d = tasks.front_tof.get_distance()
        #             if d is not None:
        #                 readings.append(d)
        #         read=readings[-10:]
        #         read.sort()
        #         mean_scan=read[5]
        #         if mean_scan<c.DISTANCE_TO_SCAN_PICK_UP:
        #             scantime=now-scantime
        #             robot.mode_extra_specific="move_straight"
        #             robot.forward(motor3,motor4,c.DRIVE_SPEED/2,c.DRIVE_SPEED/2)
        #             movetime=now
        #             time_count=now+500
                                
                                
        # elif robot.mode_extra_specific=="move_straight":
        #     if now>=time_count and motor3.return_state()==1:
        #         time_count+=500
        #         robot.forward(motor3,motor4,c.DRIVE_SPEED/2, c.DRIVE_SPEED/2, reverse=True)
        #     elif now>=time_count and motor3.return_state()==-1:
        #         # Go back to scan if see nothing
        #         robot.mode_extra_specific="scan"
        #         robot.spin(motor3, motor4, "left", c.TURN_SPEED/2)
            
        #     if tasks.front_tof.poll():
        #         d = tasks.front_tof.get_distance()
        #         if d is not None:
        #             readings.append(d)
        #         read=readings[-10:]
        #         read.sort()
        #         mean_scan=read[5]
                        
        #     if mean_scan<c.DISTANCE_TO_PICK_UP:
        #         movetime=now-movetime
        #         robot.mode_extra_specific="pick_up_grab_start"
        #         robot.stop(motor3,motor4)
                    
        # elif robot.mode_extra_specific=="pick_up_grab_start":
        #     pole = tasks.read_pole()
        #     if pole:
        #         level = tasks.level_for_pole(pole)
        #         robot.drop_off_info[1] = 1 if level == "TOP" else 0
        #         print("box magnet: %s (strongest field %s mV) -> %s level" % (
        #         pole if pole else "NOT FOUND", tasks.last_field_mv, level))
        #     else:
        #         # In case it can't see it
        #         print("CANT SEE")
        #         robot.mode="move_straight"
        #         robot.forward(motor3,motor4,c.DRIVE_SPEED/2, c.DRIVE_SPEED/2)
        #         time_count=now+500
        #         pass
        #                 # Engage servos
                        
        #     print("SERVO1 set angle:" +str(10))
        #     servo1.set_angle(10)
        #     utime.sleep_ms(500)
        #     print("SERVO2 set angle:" +str(10))
        #     servo2.set_angle(10)
        #     utime.sleep_ms(500)
                        
        #                 # Do the whole servo grabbing motion
        #     robot.mode_extra_specific="return_straight"
        #     time_count=now+movetime%500
        #     robot.forward(motor3,motor4, c.DRIVE_SPEED/2, c.DRIVE_SPEED/2, reverse=True)
                        
        # elif robot.mode_extra_specific=="return_straight":
        #     if now>=time_count:
        #         robot.mode_extra_specific="return turn"
        #         time_count=now+scantime%c.SCAN_TURN_TIME
        #         if scantime%(c.SCAN_TURN_TIME*4)+c.SCAN_TURN_TIME<c.SCAN_TURN_TIME*2:
        #             robot.spin(motor3,motor4,"right", c.TURN_SPEED/2)
        #         else:
        #             robot.spin(motor3,motor4,"left", c.TURN_SPEED/2)
        # elif robot.mode_extra_specific=="return_turn":  
        #     if now>=time_count:
        #         map[robot.final_objective]["type"]="COLLECTED"
        #         robot.mode=""
        #         robot.has_box=True
        #         picking_up=False
                    
        #             # This should have the grabbing and so on code
        #             # robot.drop_off_info[1]=0
        #             # CHANGED (2026-10-20): the fixed 0 above was a placeholder for
        #             # the rack level. The level now comes from the magnet in the
        #             # box: read_pole() watches the magnet sensor for
        #             # config.MAG_READ_MS and returns "north", "south" or None;
        #             # config.MAG_POLE_LEVEL turns that into "TOP" / "BOTTOM"
        #             # (MAG_NO_POLE_LEVEL if no magnet was found). "deliver" reads
        #             # drop_off_info[1]: 0 -> BOTTOM_ slots, 1 -> TOP_ slots.
                    
                    
                        
                
                    
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
    

    return robot, map, motor3, motor4, tasks, now, holdoff_until, skip_forward, reversing, time_count, turning, drop_off_turn_count, at_start, picking_up, readings, scantime, movetime, servo1, servo2, dropping_off, left_old_line