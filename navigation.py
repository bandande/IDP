"""
How the robot drives from node to node on the table (nodes: track_map.py).

This file only MOVES the robot. What to do and where to go is decided in
mission.py, which calls three functions from here:
    go_to(node)        drive to a node (junction, pad or rack slot) and stop
    face_slot(slot)    drive to a rack slot's junction and turn to look down
                       its rung, without driving in (to check the slot first)
    enter_slot(slot)   then drive along the rung up to the rack

THE ROBOT
  * Two driven wheels at the back. Four line sensors in a row at the front,
    LINE_SENSOR_AHEAD_M (config.py) in front of the wheel axle:
        outer-left   inner-left   inner-right   outer-right
  * The two INNER sensors keep the robot on the line: both see white when the
    line is in the middle. If only one sees it, the robot steers towards it.
  * The two OUTER sensors see lines going off to the side: a junction.

WHERE THE ROBOT IS - three things kept up to date
  node          the last node the robot reached ("START", "W3", ...)
  heading       the compass direction it is facing ("north", ...)
  sensors_past  how far (m) the line sensors are past `node`, along `heading`:
                  0                    the sensors have just reached the node
                  LINE_SENSOR_AHEAD_M  the robot has just turned on the spot
                                       on the node (wheels on the node)
  sensors_past is needed because the sensors are far in front of the wheels:
    * Before turning on the spot at `node`, the robot first drives
      (LINE_SENSOR_AHEAD_M - sensors_past) further, so that its WHEELS are on
      the junction. Then it turns around the junction and the sensors land
      on the new line.
    * Right after a turn, a node that is closer than sensors_past is
      already BEHIND the sensors, so they will never see it. It is counted
      straight away. Example: the second pad stub is ~17 cm after the first,
      the sensors are ~20 cm in front of the wheels; and the start box's
      edges are only 8-15 cm from START.

ONE STEP (move_to_neighbour) from node A to the next node B on the route
  1. If A is a pad: the robot never drove into the pad - it only turned on
     the spot at the pad's junction B to face it - so its wheels are still
     on B. Nothing to drive; the next step simply turns on the spot.
     If A is a rack slot: reverse straight back along the rung until all
     four sensors see the line it branches off. That is B.
  2. Otherwise turn on the spot to face B, if needed.
  3. If the line A -> B has the traffic light: wait for green.
  4. Get to B:
       B is a pad        -> it is right in front of the robot now: done.
       B is a rack slot  -> follow the rung until the line ends.
       B is a junction   -> follow the line until an outer sensor sees B's
                            side line (or count it at once if it is already
                            behind the sensors, see above).

STOPPING
  Every control loop checks the start/stop button (a front crash sensor).
  A press stops the motors and raises StopRequested. Anything unexpected
  (line lost, no line found while turning, ...) stops the motors and raises
  NavigationError with a message. main.py catches both.
"""

from utime import ticks_ms, ticks_diff, sleep_ms

import config
import track_map
from sensors.line_processing import OUTER_LEFT, INNER_LEFT, INNER_RIGHT, OUTER_RIGHT

BOTH_INNER = INNER_LEFT | INNER_RIGHT
ALL_FOUR = OUTER_LEFT | INNER_LEFT | INNER_RIGHT | OUTER_RIGHT


class StopRequested(Exception):
    """The start/stop button was pressed while the robot was moving."""


class NavigationError(Exception):
    """Something went wrong while driving (the message says what)."""


class Navigator:
    def __init__(self, drive, line_sensors, button, tasks):
        """drive: actuators.motor.DriveBase; line_sensors:
        sensors.line_sensor.LineSensorArray; button:
        control.start_stop.StartStopButton; tasks:
        control.task_sensors.TaskSensors (used for the traffic light)."""
        self.drive = drive
        self.line = line_sensors
        self.button = button
        self.tasks = tasks
        self.reset_at_start()

    def reset_at_start(self):
        """The robot stands in the start box with its line sensors on START,
        facing north (towards the ramp)."""
        self.node = "START"
        self.heading = "north"
        self.sensors_past = 0.0
        self._correction = None    # last steering correction: None, "left" or "right"

    # ------------------------------------------------------------------
    # Main entry point
    # ------------------------------------------------------------------
    def go_to(self, goal):
        """Drive from the current node to `goal` along the shortest route,
        then stop."""
        route = track_map.find_route(self.node, goal)
        print("route: %s" % " -> ".join(route))
        for next_node in route[1:]:
            self.move_to_neighbour(next_node)
        self.drive.stop()

    def face_slot(self, slot):
        """Drive to the junction of a rack slot and turn to look down its
        rung, without driving in. The front distance sensor then looks into
        the slot from about the length of the rung away, where it measures
        reliably (closer than ~3 cm it cannot), and a full slot costs no
        driving in and out."""
        junction = list(track_map.NEIGHBOURS[slot].values())[0]
        self.go_to(junction)
        turn = track_map.turn_needed(self.heading, track_map.direction(junction, slot))
        if turn != "straight":
            self.turn(turn)
        self.drive.stop()
        print("  at %s, facing %s towards %s" % (self.node, self.heading, slot))

    def enter_slot(self, slot):
        """After face_slot(slot): follow the rung up to its end, in front of
        the rack slot."""
        self.follow_to_line_end()
        self.node = slot
        self.sensors_past = 0.0
        print("  at %s" % slot)

    def move_to_neighbour(self, next_node):
        """One step of a route: from self.node to the neighbouring next_node."""
        here = self.node

        # 1a. Leaving a pad: the wheels are still on its junction (the robot
        #     only turned to face the pad), so there is nothing to drive.
        if here in track_map.PADS:
            self.node = next_node
            self.sensors_past = config.LINE_SENSOR_AHEAD_M
            return
        # 1b. Leaving a rack slot: back out along the rung.
        if track_map.is_dead_end(here):
            self.reverse_to_junction()
            self.node = next_node
            self.sensors_past = 0.0
            return

        # 2. Face the next node.
        turn = track_map.turn_needed(self.heading, track_map.direction(here, next_node))
        if turn != "straight":
            self.turn(turn)

        # 3. Traffic light on this line?
        if (here, next_node) in config.TRAFFIC_LIGHT_EDGES:
            self.wait_for_green()

        # 4. Get there.
        if next_node in track_map.PADS:
            pass                                  # the pad is right in front of the robot
        elif next_node in track_map.SLOTS:
            self.follow_to_line_end()
            self.sensors_past = 0.0
        else:
            gap = track_map.distance(here, next_node)
            if gap <= self.sensors_past:
                print("  %s is already behind the line sensors" % next_node)
                self.sensors_past -= gap
            else:
                # The hold-off (see follow_to_junction) is only needed when
                # the sensors start ON a node, i.e. when the robot is driving
                # straight on from a junction it has just seen.
                self.follow_to_junction(next_node, holdoff=self.sensors_past < 0.01)
                self.sensors_past = 0.0
        self.node = next_node
        print("  at %s, facing %s" % (self.node, self.heading))

    # ------------------------------------------------------------------
    # Reading the sensors (every control loop goes through here)
    # ------------------------------------------------------------------
    def read(self):
        """Pause one loop, check the stop button, return the line sensor bits."""
        sleep_ms(config.NAV_LOOP_MS)
        if self.button.pressed():
            self.drive.stop()
            raise StopRequested()
        return self.line.read()

    def _fail(self, message):
        self.drive.stop()
        raise NavigationError("%s (at %s, facing %s)" % (message, self.node, self.heading))

    # ------------------------------------------------------------------
    # Line following
    # ------------------------------------------------------------------
    def steer(self, bits):
        """One line-following step. Both inner sensors white -> straight on.
        Only the left one white -> the line is to the left -> slow the left
        wheel so the robot turns left (and the other way round). Neither ->
        keep the last correction: the line has just slipped out from between
        the two sensors, on the side we were already steering towards."""
        left_sees = bits & INNER_LEFT
        right_sees = bits & INNER_RIGHT
        if left_sees and right_sees:
            self._correction = None
        elif left_sees:
            self._correction = "left"
        elif right_sees:
            self._correction = "right"
        fast = config.NAV_DRIVE_SPEED
        slow = fast * config.NAV_STEER_SLOW_PERCENT / 100
        if self._correction == "left":
            self.drive.drive(slow, fast)
        elif self._correction == "right":
            self.drive.drive(fast, slow)
        else:
            self.drive.drive(fast, fast)

    def follow_to_junction(self, target, holdoff=True):
        """Follow the line until the outer sensors see `target`'s side line.

        From the map we know on which side(s) `target` has a line going off,
        so only an outer sensor on such a side counts. It counts when it
        CHANGES from black to white, so a side line it is already on at the
        start does not count.
        holdoff=True: also ignore the outer sensors for the first
        NAV_JUNCTION_HOLDOFF_MS. Use it when the sensors start on the node we
        are leaving: while they slide off its 2 cm wide side line they can
        flicker black-white-black, which must not count as the next node.
        After a turn the sensors are far from that node, and a hold-off
        could make them drive over a node that is only a few cm ahead."""
        want_left, want_right = track_map.side_branches(target, self.heading)
        if not (want_left or want_right):        # e.g. a T-bar end: accept either side
            want_left = want_right = True
        start = ticks_ms()
        last_white = start
        previous = self.line.read()
        while True:
            bits = self.read()
            now = ticks_ms()
            self.steer(bits)
            if bits:
                last_white = now
            elif ticks_diff(now, last_white) > config.NAV_LINE_LOST_MS:
                self._fail("lost the line on the way to %s" % target)
            if not holdoff or ticks_diff(now, start) >= config.NAV_JUNCTION_HOLDOFF_MS:
                new_left = (bits & OUTER_LEFT) and not (previous & OUTER_LEFT)
                new_right = (bits & OUTER_RIGHT) and not (previous & OUTER_RIGHT)
                if (want_left and new_left) or (want_right and new_right):
                    return                       # keep driving: the next step decides
            if ticks_diff(now, start) > config.NAV_LEG_TIMEOUT_MS:
                self._fail("no junction found on the way to %s" % target)
            previous = bits

    def follow_to_line_end(self):
        """Follow a rung until the line ends (all four sensors black for
        NAV_LINE_END_MS), then stop: the robot is in front of a rack slot."""
        start = ticks_ms()
        black_since = None
        while True:
            bits = self.read()
            now = ticks_ms()
            if bits:
                black_since = None
                self.steer(bits)
            elif black_since is None:
                black_since = now
            elif ticks_diff(now, black_since) >= config.NAV_LINE_END_MS:
                break
            if ticks_diff(now, start) > config.NAV_LEG_TIMEOUT_MS:
                self._fail("the rung did not end")
        self.drive.stop()

    # ------------------------------------------------------------------
    # Turning and reversing
    # ------------------------------------------------------------------
    def drive_wheels_onto_node(self):
        """Drive straight on until the WHEELS are on the current node (the
        sensors are LINE_SENSOR_AHEAD_M - sensors_past ahead of them)."""
        distance = config.LINE_SENSOR_AHEAD_M - self.sensors_past
        if distance > 0.005:
            ms = int(config.NAV_NODE_FORWARD_MS * distance / config.LINE_SENSOR_AHEAD_M)
            speed = config.NAV_DRIVE_SPEED
            self.drive.drive(speed, speed)
            start = ticks_ms()
            while ticks_diff(ticks_ms(), start) < ms:
                self.read()
        self.drive.stop()
        self.sensors_past = config.LINE_SENSOR_AHEAD_M

    def turn(self, turn):
        """Turn on the spot at the current node: "left", "right" or "about".

        1. Drive on until the wheels are on the junction.
        2. Spin (one wheel forwards, one backwards) for at least the minimum
           time (so the old line and any lines in between are passed), then
           stop as soon as both inner sensors see a line: the new one.
        "about" spins clockwise (to the right)."""
        self.drive_wheels_onto_node()
        if self.node == "START":
            min_ms = config.NAV_START_TURN_MIN_MS
        elif turn == "about":
            min_ms = config.NAV_TURN_MIN_MS_ABOUT
        else:
            min_ms = config.NAV_TURN_MIN_MS
        speed = config.NAV_TURN_SPEED
        if turn == "left":
            self.drive.drive(-speed, speed)
        else:
            self.drive.drive(speed, -speed)
        start = ticks_ms()
        left_old_line = False
        while True:
            bits = self.read()
            elapsed = ticks_diff(ticks_ms(), start)
            centre = bits & BOTH_INNER
            if centre == 0:
                left_old_line = True
            if elapsed >= min_ms and left_old_line and centre == BOTH_INNER:
                break
            if elapsed > min_ms + config.NAV_TURN_TIMEOUT_MS:
                self._fail("no line found while turning %s" % turn)
        self.drive.stop()
        self.heading = track_map.turn_result(self.heading, turn)
        self.sensors_past = config.LINE_SENSOR_AHEAD_M
        self._correction = None
        print("  turned %s, now facing %s" % (turn, self.heading))

    def reverse_to_junction(self):
        """Reverse straight out of a rack slot until all four sensors see the
        line the rung branches off (it crosses the whole sensor row).

        The robot arrived along the rung, so it is straight and does not need
        to steer while reversing. All four white only counts after the
        sensors have been back on the rung itself (inner sensors only), so
        nothing white at the very end of the rung can stop it too early."""
        speed = config.NAV_REVERSE_SPEED
        self.drive.drive(-speed, -speed)
        start = ticks_ms()
        on_stub = False
        while True:
            bits = self.read()
            if bits & BOTH_INNER and not bits & (OUTER_LEFT | OUTER_RIGHT):
                on_stub = True
            if on_stub and bits == ALL_FOUR:
                break
            if ticks_diff(ticks_ms(), start) > config.NAV_REVERSE_TIMEOUT_MS:
                self._fail("reversing: the sensors never all saw the line")
        self.drive.stop()
        print("  reversed out to the line")

    # ------------------------------------------------------------------
    # Traffic light
    # ------------------------------------------------------------------
    def wait_for_green(self):
        """Stand still until the traffic light (IR receiver) says green."""
        self.drive.stop()
        self.tasks.light_reset()
        print("  waiting for the traffic light")
        last_note = ticks_ms()
        while True:
            self.read()                          # keeps the stop button working
            if self.tasks.light_update() == "green":
                print("  traffic light green after %d ms" % self.tasks.light_waited_ms())
                return
            waited = self.tasks.light_waited_ms()
            if config.TRAFFIC_WAIT_MAX_MS and waited >= config.TRAFFIC_WAIT_MAX_MS:
                print("  traffic light: waited %d ms - going anyway" % waited)
                return
            if ticks_diff(ticks_ms(), last_note) >= 3000:
                last_note = ticks_ms()
                print("  still red (%d s)" % (waited // 1000))
