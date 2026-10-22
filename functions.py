from MAP import MAP
import constants as c
from collections import deque
from motor import Motor
from sensors.line_processing import OUTER_LEFT, INNER_LEFT, INNER_RIGHT, OUTER_RIGHT
import utime
import config

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
            

