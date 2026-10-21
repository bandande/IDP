"""
Link between the fake MicroPython modules (machine, utime, micropython) and
the simulated world.

sim.world.World registers itself here as `world`. The fakes then ask it for
the simulated time, for the level of an input pin (line sensors, crash
sensors), and tell it about outputs (motor DIR pins, PWM duty).
With no world registered the fakes still work on their own (time only moves
when the code sleeps; inputs read their last stored value).
"""

world = None
