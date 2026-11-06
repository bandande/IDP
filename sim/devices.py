class SimToF:
    name = "front (simulated)"
    available = True
    last_error = None

    def __init__(self, world, facing="front"):
        self.world = world
        self.facing = facing
        self.name = "%s (simulated)" % facing

    def poll(self, now_ms=None):
        return True

    def get_distance(self, now_ms=None):
        if self.facing == "rear":
            return self.world.rear_distance_mm()
        return self.world.front_distance_mm()

    @property
    def distance_mm(self):
        return self.get_distance()

    def stop(self):
        pass
