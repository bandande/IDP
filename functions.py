from MAP import MAP
import constants
from collections import deque
from motor import Motor

def degrees_to_time(degrees):
    count=0
    count=constants.full_turn_time*degrees/360
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
        self.position = "JUNCTION_LEFT_BOTTOM"
        self.direction = "west"
        

        self.previous_position = "START"
        # self.position = "START"
        # self.direction = "north"
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
                return path

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
    

        
    




