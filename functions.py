from MAP import MAP
import constants
from collections import deque

def degrees_to_time(degrees):
    count=0
    count=constants.full_turn_time*degrees/360
    return count

def turn_to_direction(direction, turn):
    directions = ["north", "east", "south", "west"]

    if direction not in directions:
        raise ValueError(f"Invalid direction: {direction}")

    if turn == "left":
        change = -1
    elif turn == "right":
        change = 1
    elif turn == "about":
        change = 2
    else:
        raise ValueError(f"Invalid turn: {turn}")

    return directions[(directions.index(direction) + change) % 4]

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
    def __init__(self):
        self.position = "START"
        self.previous_position = None
        self.direction = "north"
        self.objective = None
        self.mode = None
        self.mode_store = None
        self.mode_specific = None
        # mode=(collecting, turning, stopping, straight, dropping)
        # specific_mode=()
        
        self.queue = [] # Macro is the largest. It is as follows- first element is the name, next elements are instructions
        
        self.left_sensor = True
        self.right_sensor = True
        self.front_left_sensor = True
        self.front_right_sensor = True
        
        self.has_box = False
        self.box_count = 0

    
    # Detection
    
    def detect_junction(self):
        pass
    
    # Moving
    
    def left_turn(self):
        pass
    
    def right_turn(self):
        pass
    
    def about_turn(self):
        pass
    
    def small_left_turn(self, degree):
        pass
    
    def small_right_turn(self, degree):
        pass
        
    
    def find_route(self, location):
        # Check that both locations exist
        if self.position not in MAP or location not in MAP:
            return None

        # Queue of locations to explore
        queue = deque([self.position])

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
        return None
    
    def move_to(self, location):
        # From one point to adjacent point
        if MAP.get(self.position) and MAP.get(location):
            final_direction=MAP[self.position].keys()[MAP[self.position].values().index(location)]
            turning=direction_to_turn(self.direction, final_direction)
        else:
            print("error, location or own position not found")
        
        
        self.objective = location
        if turning=="straight":
            return ["straight"]
        else:
            return ["straight", ["turning", turning, "90"]]
    
    
robot=Robot()
print(robot.find_route("RED_2"))
        
    




