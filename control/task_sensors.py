"""
Sensors used by the TASK logic in main.py (line following does not need them).

  * Magnet pole of the box being picked up (Hall sensor, analog) -> which rack
    level (TOP / BOTTOM) the box must go to.          read_pole(), level_for_pole()
  * Front distance sensor (one VL53L0X, config.FRONT_TOF) at a drop slot ->
    is there already a box in the slot?               slot_occupied()
  * Grab distance sensor (the second VL53L0X, config.GRAB_TOF): how far away
    is the box to pick up? For aiming at it and driving up to it.
                                                      box_distance_now(), box_distance(),
                                                      box_seen(), box_in_reach()
  * IR receiver at the traffic light -> may the robot go?
                                                      light_reset(), light_update()

Every sensor is optional: if one does not start (not wired, I2C error, ...)
the matching check falls back to a safe default and a warning is printed,
so the rest of the task still runs:
  * no magnet sensor / no magnet found -> config.MAG_NO_POLE_LEVEL
  * no front distance sensor / no reading -> the slot is treated as FREE
  * no grab distance sensor -> box distances are None (no box seen); the
    caller decides what to do then (e.g. drive a fixed time instead)
  * no IR receiver -> the light is treated as GREEN

Usage (see main.py)
    tasks = TaskSensors.from_config()
    tasks.calibrate()                    # start-up, no magnets near the robot
    pole = tasks.read_pole()             # "north", "south" or None (blocking, MAG_READ_MS)
    level = tasks.level_for_pole(pole)   # "TOP" or "BOTTOM"
    full = tasks.slot_occupied()         # True / False (blocking, SLOT_READ_MS)
    d = tasks.box_distance_now()         # grab sensor, every loop while moving: mm or None
    if tasks.box_in_reach(d): ...        # close enough to grab (config.GRAB_REACH_MM)
    tasks.light_reset()                  # when the robot stops at the light
    state = tasks.light_update()         # every loop: "red" or "green"

The factory functions make_magnet / make_front_tof / make_grab_tof / make_ir
build the real sensors; the PC simulator (sim/) replaces them with
simulated ones.
"""

from utime import ticks_ms, ticks_diff, sleep_ms

import config


def make_magnet():
    from sensors.magnetic import HallSensor
    return HallSensor.from_config()


def _start_tof(name, role):
    """Start only the VL53L0X called `name` in config.TOF_SENSORS (`role` is
    used in error messages). The two sensors are on separate I2C buses, so
    each can be started on its own."""
    from sensors.tof import setup_tof_sensors
    specs = [spec for spec in config.TOF_SENSORS if spec["name"] == name]
    if not specs:
        raise ValueError("%s distance sensor %r is not in config.TOF_SENSORS" % (role, name))
    sensors = setup_tof_sensors(specs, config.I2C_FREQ_HZ,
                                max_valid_mm=config.TOF_MAX_VALID_MM,
                                median_size=config.TOF_MEDIAN_SIZE,
                                long_range=config.TOF_LONG_RANGE,
                                stale_ms=config.TOF_STALE_MS)
    tof = sensors[name]
    if not tof.available:
        raise OSError("%s distance sensor %s did not start: %s" % (role, name, tof.last_error))
    return tof


def make_front_tof():
    """Start only the forward-facing VL53L0X (config.FRONT_TOF): drop slot check."""
    return _start_tof(config.FRONT_TOF, "front")


def make_grab_tof():
    """Start only the second VL53L0X (config.GRAB_TOF): aiming at a box to pick up."""
    return _start_tof(config.GRAB_TOF, "grab")


def make_ir():
    from sensors.ir_receiver import IRReceiver
    return IRReceiver.from_config()


def _median(values):
    values = sorted(values)
    return values[len(values) // 2]


class TaskSensors:
    def __init__(self, magnet=None, front_tof=None, ir=None, grab_tof=None):
        self.magnet = magnet
        self.front_tof = front_tof
        self.ir = ir
        # The grab sensor has the same interface as front_tof (poll(),
        # get_distance(), distance_mm), so code written for one works with
        # the other: tasks.grab_tof.poll(), tasks.grab_tof.get_distance().
        self.grab_tof = grab_tof
        self.last_field_mv = None     # strongest field seen by the last read_pole()
        self.last_slot_mm = None      # median distance of the last slot_occupied()
        self.last_box_mm = None       # median distance of the last box_distance()
        self._ir_seen = None          # light_update(): time of the last IR activity
        self._ir_on_since = None      # light_update(): start of the current unbroken IR stretch
        self._light_start = None
        self._last_light = None

    @classmethod
    def from_config(cls):
        """Start the four task sensors; any that fails is left out (warning printed)."""
        parts = []
        for name, factory in (
                ("magnet sensor", make_magnet),
                ("front distance sensor", make_front_tof),
                ("IR receiver", make_ir),
                ("grab distance sensor", make_grab_tof),
            ):
            try:
                parts.append(factory())
            except Exception as e:   # a missing sensor must not stop the robot
                print("WARNING: %s not available (%r) - using its fallback" % (name, e))
                parts.append(None)
        return cls(*parts)

    # -- magnet ---------------------------------------------------------------
    def calibrate(self):
        """Measure the magnet sensor's zero-field baseline. Call at start-up with
        NO magnet near the robot (e.g. in the start box). Returns mV or None."""
        if self.magnet is None or self.magnet.mode != "analog":
            return None
        return self.magnet.calibrate_baseline()

    def read_pole(self, read_ms=None):
        """Watch the magnet sensor for read_ms (default config.MAG_READ_MS) and
        return "north", "south", or None if no magnet was strong enough.

        The strongest reading in the window decides: |field| must reach
        config.MAG_ON_MV; its sign gives the pole (config.MAG_NORTH_SIGN = the
        sign a NORTH pole gives on this robot). Blocking; the robot is stopped."""
        if self.magnet is None:
            return None
        if read_ms is None:
            read_ms = config.MAG_READ_MS
        best = 0
        start = ticks_ms()
        while ticks_diff(ticks_ms(), start) < read_ms:
            self.magnet.update()
            field = self.magnet.field_mv
            if field is not None and abs(field) > abs(best):
                best = field
            sleep_ms(5)
        self.last_field_mv = best
        if abs(best) < config.MAG_ON_MV:
            return None
        return "north" if (best > 0) == (config.MAG_NORTH_SIGN > 0) else "south"

    @staticmethod
    def level_for_pole(pole):
        """Rack level ("TOP" / "BOTTOM") for a pole, from config.MAG_POLE_LEVEL."""
        if pole is None:
            return config.MAG_NO_POLE_LEVEL
        return config.MAG_POLE_LEVEL[pole]

    # -- front distance sensor -------------------------------------------------
    def slot_occupied(self, read_ms=None):
        """Is there a box in the slot straight ahead? Collects distance readings
        for read_ms (default config.SLOT_READ_MS) and compares their median with
        config.SLOT_OCCUPIED_MM. No sensor or no valid reading -> False (free).
        Blocking; the robot is stopped in front of the slot."""
        self.last_slot_mm = None
        if self.front_tof is None:
            return False
        if read_ms is None:
            read_ms = config.SLOT_READ_MS
        readings = []
        start = ticks_ms()
        while ticks_diff(ticks_ms(), start) < read_ms:
            if self.front_tof.poll():
                d = self.front_tof.get_distance()
                if d is not None:
                    readings.append(d)
            sleep_ms(5)
        if not readings:
            return False      # nothing within range: the slot is open / empty
        self.last_slot_mm = _median(readings)
        return self.last_slot_mm < config.SLOT_OCCUPIED_MM

    # -- grab distance sensor: aiming at a box to pick up ------------------------
    # How exactly the robot aims is still being decided. A typical use:
    #   * while turning slowly from side to side, call box_distance_now() every
    #     loop; the heading with the SMALLEST distance points at the box;
    #   * then creep forwards, calling box_distance_now() every loop, and stop
    #     as soon as box_in_reach(distance) is True;
    #   * standing still, box_distance() gives a steadier value (a median).
    # Note: the VL53L0X does not measure reliably closer than about 30 mm, so
    # check what it really reads at grabbing distance before setting
    # config.GRAB_REACH_MM.
    def box_distance_now(self):
        """Non-blocking: the newest filtered distance (mm) from the grab sensor,
        or None (nothing in range, reading too old, or no grab sensor). Call it
        every loop while the robot moves (the sensor gives ~30 new values / s)."""
        if self.grab_tof is None:
            return None
        self.grab_tof.poll()
        return self.grab_tof.get_distance()

    def box_distance(self, read_ms=None):
        """Blocking: the median distance (mm) seen by the grab sensor during
        read_ms (default config.GRAB_READ_MS), or None if nothing was in range.
        Use it while the robot stands still."""
        self.last_box_mm = None
        if self.grab_tof is None:
            return None
        if read_ms is None:
            read_ms = config.GRAB_READ_MS
        readings = []
        start = ticks_ms()
        while ticks_diff(ticks_ms(), start) < read_ms:
            if self.grab_tof.poll():
                d = self.grab_tof.get_distance()
                if d is not None:
                    readings.append(d)
            sleep_ms(5)
        if readings:
            self.last_box_mm = _median(readings)
        return self.last_box_mm

    @staticmethod
    def box_seen(distance_mm):
        """True if a distance from the grab sensor means "a box is in front"
        (closer than config.GRAB_SEE_MM)."""
        return distance_mm is not None and distance_mm < config.GRAB_SEE_MM

    @staticmethod
    def box_in_reach(distance_mm):
        """True if a distance from the grab sensor means "close enough to grab"
        (config.GRAB_REACH_MM or closer)."""
        return distance_mm is not None and distance_mm <= config.GRAB_REACH_MM

    # -- traffic light ---------------------------------------------------------
    def light_reset(self):
        """Call once when the robot stops at the traffic light."""
        self._light_start = ticks_ms()
        self._ir_seen = None          # time of the last IR activity since the reset
        self._ir_on_since = None      # start of the current unbroken stretch of IR activity
        self._last_light = None
        if self.ir is not None:
            self.ir.process()         # drop edges captured while nobody was looking
            self.ir.clear()

    def light_update(self):
        """Call every loop while waiting. Returns "red" or "green".

        config.TRAFFIC_RED_WHEN_IR = False (our light: IR beacon = GREEN):
        green once IR activity has been seen WITHOUT A BREAK for
        config.TRAFFIC_IR_CONFIRM_MS. signal_present() stays true for
        IR_PRESENT_WINDOW_MS (150 ms) after each burst, so a beacon sending
        bursts keeps it true, while one stray pulse (noise) cannot last the
        200 ms confirmation. No IR = red, keep waiting.
        config.TRAFFIC_RED_WHEN_IR = True: the other way round - IR means RED,
        and the light counts as GREEN only after config.TRAFFIC_GREEN_AFTER_MS
        without any IR (longer than a blinking beacon's off phase).
        No IR receiver -> always "green"."""
        if self.ir is None:
            return "green"
        now = ticks_ms()
        if self._light_start is None:
            self.light_reset()
        if self.ir.signal_present():
            self._ir_seen = now
            if self._ir_on_since is None:
                self._ir_on_since = now
        else:
            self._ir_on_since = None
        if config.TRAFFIC_RED_WHEN_IR:
            last = self._light_start if self._ir_seen is None else self._ir_seen
            state = "green" if ticks_diff(now, last) >= config.TRAFFIC_GREEN_AFTER_MS else "red"
        else:
            steady = (self._ir_on_since is not None
                      and ticks_diff(now, self._ir_on_since) >= config.TRAFFIC_IR_CONFIRM_MS)
            state = "green" if steady else "red"
        self._last_light = state
        return state

    def light_waited_ms(self):
        """How long the robot has been waiting at the light (ms)."""
        if self._light_start is None:
            return 0
        return ticks_diff(ticks_ms(), self._light_start)
