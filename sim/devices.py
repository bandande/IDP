"""
Simulated stand-ins for sensors that are not driven through fake pins.

SimToF replaces a VL53L0X: its distance is a ray cast into the simulated
table (boxes in rack slots / on pads and the table edge; empty slots are
open). CHANGED (2026-10-27): facing="front" (default) casts from the nose
along the heading (the grab sensor, control.task_sensors.make_grab_tof);
facing="rear" casts from the tail backwards (the rear sensor, make_rear_tof).
The magnet sensor (ADC) and the IR receiver (pin edges) use the real driver
code; the World feeds their pins.
"""


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
