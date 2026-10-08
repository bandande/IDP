


class Robot:
    def __init__(self):
        self.position = "start"
        self.previous_position = None
        self.direction = "north"
        self.objective = None
        self.mode = None

    def move_to(self, location):
        self.previous_position = self.position
        self.position = location




