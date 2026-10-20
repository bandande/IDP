"""
Simulated stand-ins for sensors that are not driven through fake pins.

SimToF replaces the forward VL53L0X (control.task_sensors.make_front_tof):
its distance is a ray cast from the robot's nose into the simulated table
(boxes in rack slots and the table edge; empty slots are open).
The magnet sensor (ADC) and the IR receiver (pin edges) use the real driver
code; the World feeds their pins.
"""


class SimToF:
    name = "front (simulated)"
    available = True
    last_error = None

    def __init__(self, world):
        self.world = world

    def poll(self, now_ms=None):
        return True

    def get_distance(self, now_ms=None):
        return self.world.front_distance_mm()

    @property
    def distance_mm(self):
        return self.get_distance()

    def stop(self):
        pass
