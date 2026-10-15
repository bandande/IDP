"""
Backup line-following controller (pure logic, no hardware).

Feed it the line sensor bitmask every loop; it returns the left and right
wheel speeds (percent, -100 .. +100). The robot code then passes them to
actuators.motor.DriveBase. Because nothing here touches hardware, the whole
behaviour is unit tested on a PC.

States
  IDLE          not started (or stopped): speeds 0, 0
  FOLLOW        PD steering on sensors.line_processing.LineTracker's error:
                    correction = kp * error + kd * d(error)/dt   (clamped)
                    left = base + correction, right = base - correction
                error < 0 means the line is left of centre, so the left
                wheel slows down and the robot turns left.
  SEARCH        the line was lost: spin towards the side it was last seen,
                back to FOLLOW when an inner sensor finds it, DONE after
                search_timeout_ms.
  TURN_FORWARD  a turn was decided at a junction: keep driving straight
                until turn_forward_ms after the junction was first seen, so
                the wheels are over the junction before spinning.
  TURN_SPIN     spin on the spot towards the new branch until an inner
                sensor finds it (after leaving the old line if the old line
                was still under the sensors), then FOLLOW again.
  DONE          finished or gave up; `reason` says why. Speeds 0, 0.

Junctions (sensors.line_processing.JunctionDetector events)
  * With a plan (list of "L", "R", "S", "stop"), one entry is used at every
    junction when it is first seen (ENTER).
  * Without a plan, or with "S": keep following. If the line then turns out
    not to continue straight (EXIT with straight == False):
      - corner (one side only): turn that way automatically,
      - T approached from the stem (both sides): t_stem_action
        ("stop", "L" or "R").
  * junction_count counts the junctions seen since start().

Usage
    from control.line_follower import LineFollower
    follower = LineFollower.from_config()
    follower.start(plan=["L", "S", "R"])        # or start() without a plan
    while True:
        left, right = follower.update(line.read())
        drive.drive(left, right)
"""

from utime import ticks_ms, ticks_diff

from sensors.line_processing import LineTracker, JunctionDetector, JunctionEvent, INNER

LEFT = "L"
RIGHT = "R"
STRAIGHT = "S"
STOP = "stop"


def _clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


class TurnWatcher:
    """Tells when a spin turn has found the new line.

    LEAVING: wait until both inner sensors have been black for leave_samples
             readings (skipped with start(already_off_line=True)).
    FINDING: wait until an inner sensor has seen white for find_samples
             readings in a row.
    """

    IDLE = "IDLE"
    LEAVING = "LEAVING"
    FINDING = "FINDING"
    DONE = "DONE"

    def __init__(self, leave_samples=2, find_samples=2):
        self.leave_samples = leave_samples
        self.find_samples = find_samples
        self.phase = TurnWatcher.IDLE
        self._count = 0

    def start(self, already_off_line=False):
        self.phase = TurnWatcher.FINDING if already_off_line else TurnWatcher.LEAVING
        self._count = 0

    def update(self, bits):
        inner = bits & INNER
        if self.phase == TurnWatcher.LEAVING:
            if inner:
                self._count = 0
            else:
                self._count += 1
                if self._count >= self.leave_samples:
                    self.phase = TurnWatcher.FINDING
                    self._count = 0
            return False
        if self.phase == TurnWatcher.FINDING:
            if inner:
                self._count += 1
                if self._count >= self.find_samples:
                    self.phase = TurnWatcher.DONE
                    return True
            else:
                self._count = 0
            return False
        return self.phase == TurnWatcher.DONE


class LineFollower:
    IDLE = "IDLE"
    FOLLOW = "FOLLOW"
    SEARCH = "SEARCH"
    TURN_FORWARD = "TURN_FORWARD"
    TURN_SPIN = "TURN_SPIN"
    DONE = "DONE"

    def __init__(self, base_speed=35, kp=35.0, kd=0.0, max_correction=45, search_speed=30,
                 search_timeout_ms=2000, turn_speed=35, turn_forward_ms=300, turn_timeout_ms=3000,
                 t_stem_action=STOP, tracker=None, junctions=None):
        if t_stem_action not in (STOP, LEFT, RIGHT):
            raise ValueError("t_stem_action must be 'stop', 'L' or 'R'")
        self.base_speed = base_speed
        self.kp = kp
        self.kd = kd
        self.max_correction = max_correction
        self.search_speed = search_speed
        self.search_timeout_ms = search_timeout_ms
        self.turn_speed = turn_speed
        self.turn_forward_ms = turn_forward_ms
        self.turn_timeout_ms = turn_timeout_ms
        self.t_stem_action = t_stem_action
        self.tracker = tracker if tracker is not None else LineTracker()
        self.junctions = junctions if junctions is not None else JunctionDetector()
        self.watcher = TurnWatcher()
        self.state = LineFollower.IDLE
        self.reason = ""
        self.error = 0.0
        self.left = 0
        self.right = 0
        self.junction_count = 0
        self.last_event = None
        self.turn_dir = None
        self.plan = []
        self._prev_error = 0.0
        self._prev_ms = None
        self._enter_ms = 0
        self._phase_ms = 0
        self._search_dir = 0

    @classmethod
    def from_config(cls):
        import config
        return cls(base_speed=config.FOLLOW_BASE_SPEED, kp=config.FOLLOW_KP, kd=config.FOLLOW_KD,
                   max_correction=config.FOLLOW_MAX_CORRECTION, search_speed=config.FOLLOW_SEARCH_SPEED,
                   search_timeout_ms=config.FOLLOW_SEARCH_TIMEOUT_MS, turn_speed=config.TURN_SPEED,
                   turn_forward_ms=config.TURN_FORWARD_MS, turn_timeout_ms=config.TURN_TIMEOUT_MS,
                   t_stem_action=config.T_STEM_ACTION,
                   tracker=LineTracker.from_config(), junctions=JunctionDetector.from_config())

    # ------------------------------------------------------------ control
    @property
    def running(self):
        return self.state not in (LineFollower.IDLE, LineFollower.DONE)

    def start(self, now_ms=None, plan=None):
        """Start following. plan: optional list of "L" / "R" / "S" / "stop", one per junction."""
        if now_ms is None:
            now_ms = ticks_ms()
        self.plan = list(plan) if plan else []
        for step in self.plan:
            if step not in (LEFT, RIGHT, STRAIGHT, STOP):
                raise ValueError("plan entries must be 'L', 'R', 'S' or 'stop'")
        self.tracker.reset()
        self.junctions.reset(counts=True)
        self.junction_count = 0
        self.last_event = None
        self.turn_dir = None
        self.reason = ""
        self.error = 0.0
        self._prev_error = 0.0
        self._prev_ms = now_ms
        self._set_state(LineFollower.FOLLOW, now_ms)

    def stop(self, reason="stopped"):
        self.state = LineFollower.IDLE
        self.reason = reason
        self.left = 0
        self.right = 0

    def _finish(self, reason):
        self.state = LineFollower.DONE
        self.reason = reason
        return self._out(0, 0)

    def _set_state(self, state, now_ms):
        self.state = state
        self._phase_ms = now_ms

    def _out(self, left, right):
        self.left = _clamp(left, -100, 100)
        self.right = _clamp(right, -100, 100)
        return self.left, self.right

    # ------------------------------------------------------------ main step
    def update(self, bits, now_ms=None):
        """One control step. Returns (left_speed, right_speed) in percent."""
        if now_ms is None:
            now_ms = ticks_ms()
        if self.state == LineFollower.FOLLOW:
            return self._follow(bits, now_ms)
        if self.state == LineFollower.SEARCH:
            return self._search(bits, now_ms)
        if self.state == LineFollower.TURN_FORWARD:
            return self._turn_forward(bits, now_ms)
        if self.state == LineFollower.TURN_SPIN:
            return self._turn_spin(bits, now_ms)
        return self._out(0, 0)

    def _follow(self, bits, now_ms):
        error = self.tracker.update(bits, now_ms)
        event = self.junctions.update(bits, now_ms)
        if event is not None:
            self.last_event = event
            if event.kind == JunctionEvent.ENTER:
                self.junction_count += 1
                self._enter_ms = now_ms
                action = self.plan.pop(0) if self.plan else None
                if action == STOP:
                    return self._finish("plan: stop at junction %d" % self.junction_count)
                if action in (LEFT, RIGHT):
                    return self._begin_turn(action, bits, now_ms)
            elif not event.straight:
                # The line does not continue straight on: corner or T from the stem.
                if event.left and not event.right:
                    return self._begin_turn(LEFT, bits, now_ms)
                if event.right and not event.left:
                    return self._begin_turn(RIGHT, bits, now_ms)
                if self.t_stem_action == STOP:
                    return self._finish("T junction from the stem")
                return self._begin_turn(self.t_stem_action, bits, now_ms)

        if self.tracker.state == LineTracker.LOST:
            sign = 1 if error > 0 else (-1 if error < 0 else 0)
            if sign == 0:
                return self._finish("line lost, side unknown")
            self._search_dir = sign
            self._set_state(LineFollower.SEARCH, now_ms)
            return self._search(bits, now_ms)

        dt_ms = ticks_diff(now_ms, self._prev_ms) if self._prev_ms is not None else 0
        derivative = (error - self._prev_error) * 1000.0 / dt_ms if dt_ms > 0 else 0.0
        self._prev_error = error
        self._prev_ms = now_ms
        self.error = error
        correction = _clamp(self.kp * error + self.kd * derivative, -self.max_correction, self.max_correction)
        return self._out(self.base_speed + correction, self.base_speed - correction)

    def _search(self, bits, now_ms):
        if bits & INNER:
            self.tracker.reset()
            self._prev_error = 0.0
            self._prev_ms = now_ms
            self._set_state(LineFollower.FOLLOW, now_ms)
            return self._follow(bits, now_ms)
        if ticks_diff(now_ms, self._phase_ms) >= self.search_timeout_ms:
            return self._finish("line lost, search timed out")
        s = self.search_speed
        return self._out(s * self._search_dir, -s * self._search_dir)

    def _begin_turn(self, direction, bits, now_ms):
        self.turn_dir = direction
        self._set_state(LineFollower.TURN_FORWARD, now_ms)
        return self._turn_forward(bits, now_ms)

    def _turn_forward(self, bits, now_ms):
        if ticks_diff(now_ms, self._enter_ms) < self.turn_forward_ms:
            return self._out(self.base_speed, self.base_speed)
        self.watcher.start(already_off_line=not (bits & INNER))
        self._set_state(LineFollower.TURN_SPIN, now_ms)
        return self._turn_spin(bits, now_ms)

    def _turn_spin(self, bits, now_ms):
        if self.watcher.update(bits):
            self.tracker.reset()
            self.junctions.reset()
            self._prev_error = 0.0
            self._prev_ms = now_ms
            self._set_state(LineFollower.FOLLOW, now_ms)
            return self._follow(bits, now_ms)
        if ticks_diff(now_ms, self._phase_ms) >= self.turn_timeout_ms:
            return self._finish("turn timed out")
        s = self.turn_speed
        if self.turn_dir == LEFT:
            return self._out(-s, s)
        return self._out(s, -s)
