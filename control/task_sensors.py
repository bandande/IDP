from utime import ticks_ms, ticks_diff, sleep_ms

import config

DEBUG_PRINT_MS = 200
TOF_POLL_MS = 25


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
                                stale_ms=config.TOF_STALE_MS,
                                period_ms=config.TOF_PERIOD_MS)   # slower, cooler
    tof = sensors[name]
    if not tof.available:
        raise OSError("%s distance sensor %s did not start: %s" % (role, name, tof.last_error))
    return tof


def make_rear_tof():
    """Start only the VL53L0X at the
    BACK (config.REAR_TOF): drop slot check while reversing into a branch."""
    return _start_tof(config.REAR_TOF, "rear")


def make_grab_tof():
    """Start only the VL53L0X at the front, through the gripper (config.GRAB_TOF)."""
    return _start_tof(config.GRAB_TOF, "grab")


def make_ir():
    from sensors.ir_receiver import IRReceiver
    return IRReceiver.from_config()


def _median(values):
    values = sorted(values)
    return values[len(values) // 2]


class TaskSensors:
    def __init__(self, magnet=None, rear_tof=None, ir=None, grab_tof=None):
        self.magnet = magnet
        self.rear_tof = rear_tof
        self.ir = ir
        # The grab sensor has the same interface as rear_tof (poll(),
        # get_distance(), distance_mm), so code written for one works with
        # the other: tasks.grab_tof.poll(), tasks.grab_tof.get_distance().
        self.grab_tof = grab_tof
        self.last_field_mv = None     # strongest field seen by the last read_pole()
        self.last_slot_mm = None      # median distance of the last rear_distance()
        self.last_rear_readings = []
        self._debug_print_ms = None
        self._ir_seen = None         # light_update(): time of the last IR activity
        self._ir_on_since = None      # light_update(): start of the current unbroken IR stretch
        self._light_start = None
        self._last_light = None

    @classmethod
    def from_config(cls):
        """Start the four task sensors; any that fails is left out (warning printed)."""
        parts = []
        for name, factory in (
                ("magnet sensor", make_magnet),
                ("rear distance sensor", make_rear_tof),
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

    # -- rear distance sensor --------------------------------------------------

    # -- grab distance sensor: aiming at a box to pick up ------------------------
    # Note: the VL53L0X does not measure reliably closer than about 30 mm, so
    # check what it really reads at grabbing distance before setting
    # config.GRAB_REACH_MM.
    def box_distance_now(self):
        """Non-blocking: the newest filtered distance (mm) from the grab sensor,
        or None (nothing in range, reading too old, or no grab sensor). Call it
        every loop while the robot moves (the sensor gives ~10 new values / s,
        one every config.TOF_PERIOD_MS)."""
        now = ticks_ms()
        print_now = self._debug_print_ms is None or ticks_diff(now, self._debug_print_ms) >= DEBUG_PRINT_MS
        if print_now:
            self._debug_print_ms = now
        if self.grab_tof is None:
            if print_now:
                print("DEBUG: grab_tof is None")
            return None

        result = self.grab_tof.poll(now)
        distance = self.grab_tof.get_distance(now)
        if print_now:
            # True = a new reading came in at this call, then the distance (mm or None)
            print("DEBUG:", result, distance)
        return distance

    def rear_distance(self, read_ms=None):
        """Blocking: the median distance (mm) seen by the
        rear sensor during read_ms (default config.SLOT_READ_MS), or None if
        nothing was in range (or there is no rear sensor). Use it while the
        robot stands still, e.g. to check a drop slot behind the robot.

        The sensor only has a NEW value every config.TOF_PERIOD_MS (100 ms),
        so the readings must be collected over some time: asking 10 times
        in a row (a few ms) gives at most one value, usually none.
        The sensor's own median filter is cleared first, so values measured
        while the robot was still turning are not used.
        Every value kept is in self.last_rear_readings (for printing)."""
        self.last_rear_readings = []
        self.last_slot_mm = None
        if self.rear_tof is None:
            print("rear distance sensor not available")
            return None
        if read_ms is None:
            read_ms = config.SLOT_READ_MS
        self.rear_tof.processor.reset()     # forget the readings from before this check
        start = ticks_ms()
        while ticks_diff(ticks_ms(), start) < read_ms:
            if self.rear_tof.poll():
                d = self.rear_tof.get_distance()
                if d is not None:
                    self.last_rear_readings.append(d)
            sleep_ms(TOF_POLL_MS)
        if self.last_rear_readings:
            self.last_slot_mm = _median(self.last_rear_readings)
        return self.last_slot_mm

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
