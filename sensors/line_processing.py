"""
Line-following data processing for the 4 x SEN0017 array (pure logic, no hardware).

A reading is a 4-bit mask; a set bit means that sensor sees WHITE (the line):

    bit 3        bit 2        bit 1         bit 0
    OUTER_LEFT   INNER_LEFT   INNER_RIGHT   OUTER_RIGHT

Assumed sensor geometry (check it on the robot):
  * The two inner sensors are closer together than the line is wide, so both
    see white when the robot is centred on the line.
  * Each outer sensor is further from its inner neighbour than the line is
    wide. An outer sensor therefore only sees white at a junction / branch,
    or when the robot has drifted so far that both inner sensors are black.
If your layout differs, pass your own `table` to LineTracker and re-check the
junction rule in JunctionDetector.

Contents:
  ERROR_TABLE, line_error()  steering error from a single reading
  LineTracker                error with memory + lost-line detection
  JunctionEvent              what JunctionDetector reports
  JunctionDetector           junction ENTER / EXIT events, shapes match MAP.py
"""

from utime import ticks_ms, ticks_diff

OUTER_LEFT = 8
INNER_LEFT = 4
INNER_RIGHT = 2
OUTER_RIGHT = 1
INNER = INNER_LEFT | INNER_RIGHT
OUTER = OUTER_LEFT | OUTER_RIGHT

# Steering error for every reading, indexed by the bitmask.
# Negative = the line is LEFT of the robot centre (steer left), positive = right.
# None = no sensor sees the line. The inner sensors decide first; the outer
# sensors only matter when both inner sensors are black (large drift).
ERROR_TABLE = (
    None,   # 0000 no line
    1.0,    # 0001 far right: only outer-right sees it
    0.4,    # 0010 slightly right
    0.7,    # 0011 right
    -0.4,   # 0100 slightly left
    -0.4,   # 0101 line under inner-left, branch on the right
    0.0,    # 0110 centred
    0.0,    # 0111 centred, branch on the right
    -1.0,   # 1000 far left: only outer-left sees it
    0.0,    # 1001 both outer only (crossing a perpendicular line), keep straight
    0.4,    # 1010 line under inner-right, branch on the left
    0.4,    # 1011 line under inner-right, junction
    -0.7,   # 1100 left
    -0.4,   # 1101 line under inner-left, junction
    0.0,    # 1110 centred, branch on the left
    0.0,    # 1111 centred on a junction / cross bar
)


def bits_to_str(bits):
    """Return e.g. "#..#" for printing: '#' = white (line), '.' = black, left to right."""
    out = ""
    for mask in (OUTER_LEFT, INNER_LEFT, INNER_RIGHT, OUTER_RIGHT):
        out += "#" if bits & mask else "."
    return out


def line_error(bits, table=ERROR_TABLE):
    """Steering error in [-1.0, 1.0] for one reading, or None if no line is seen."""
    return table[bits & 0x0F]


class LineTracker:
    """Turns readings into a steering error with memory.

    When no sensor sees the line the error is the sign of the last non-zero
    error (keep turning towards where the line was last seen). After `lost_ms`
    of continuous all-black the state becomes LOST (line really lost, or the
    end of a dead-end line).

    Attributes after update(): error, state ("ON_LINE", "SEARCHING", "LOST"),
    lost_for_ms, just_lost (True only on the update where LOST begins).
    """

    ON_LINE = "ON_LINE"
    SEARCHING = "SEARCHING"
    LOST = "LOST"

    def __init__(self, lost_ms=150, table=ERROR_TABLE):
        if len(table) != 16:
            raise ValueError("table needs 16 entries")
        self.lost_ms = lost_ms
        self.table = table
        self.reset()

    @classmethod
    def from_config(cls):
        import config
        return cls(lost_ms=config.LINE_LOST_MS)

    def reset(self):
        self.error = 0.0
        self.state = LineTracker.ON_LINE
        self.lost_for_ms = 0
        self.just_lost = False
        self._last_sign = 0
        self._lost_since = None

    def update(self, bits, now_ms=None):
        if now_ms is None:
            now_ms = ticks_ms()
        self.just_lost = False
        e = self.table[bits & 0x0F]
        if e is not None:
            self.error = e
            if e > 0:
                self._last_sign = 1
            elif e < 0:
                self._last_sign = -1
            self.state = LineTracker.ON_LINE
            self.lost_for_ms = 0
            self._lost_since = None
            return e

        if self._lost_since is None:
            self._lost_since = now_ms
        self.lost_for_ms = ticks_diff(now_ms, self._lost_since)
        self.error = float(self._last_sign)
        if self.lost_for_ms >= self.lost_ms:
            if self.state != LineTracker.LOST:
                self.just_lost = True
            self.state = LineTracker.LOST
        else:
            self.state = LineTracker.SEARCHING
        return self.error


FOUR_JUNCTION = "FOUR_JUNCTION"
T_JUNCTION = "T_JUNCTION"
CORNER = "CORNER"


def junction_shape(left, right, straight):
    """Map what was seen at a junction to the MAP.py junction type names."""
    if left and right:
        return FOUR_JUNCTION if straight else T_JUNCTION
    if left or right:
        return T_JUNCTION if straight else CORNER
    return None


class JunctionEvent:
    """Reported by JunctionDetector.

    kind         "ENTER" (junction just confirmed) or "EXIT" (robot has passed it)
    left, right  a branch was seen on that side (at ENTER: so far; at EXIT: final)
    straight     EXIT only: the line continues straight on after the junction
    shape        EXIT only: "FOUR_JUNCTION", "T_JUNCTION" or "CORNER" (MAP.py names)
    t_ms         ticks_ms() of the update that produced the event
    duration_ms  EXIT only: time spent on the junction
    """

    ENTER = "ENTER"
    EXIT = "EXIT"

    def __init__(self, kind, left, right, straight=None, shape=None, t_ms=0, duration_ms=0):
        self.kind = kind
        self.left = left
        self.right = right
        self.straight = straight
        self.shape = shape
        self.t_ms = t_ms
        self.duration_ms = duration_ms

    def __repr__(self):
        return "JunctionEvent(%s, left=%s, right=%s, straight=%s, shape=%s, t=%d, dur=%d)" % (
            self.kind, self.left, self.right, self.straight, self.shape, self.t_ms, self.duration_ms)


class JunctionDetector:
    """Detects junctions from successive readings.

    IDLE: a reading is a junction candidate when an outer sensor sees white
    while at least one inner sensor is still on the line (a pure drift gives
    1000 / 0001 instead, so it does not trigger). `confirm_samples` candidates
    in a row emit an ENTER event.

    IN JUNCTION: every outer-sensor hit is OR-ed into left / right. When both
    outer sensors have been black for `clear_samples` readings in a row an EXIT
    event is emitted with straight = "an inner sensor still sees the line".

    The sides reported at ENTER may be incomplete if the robot meets the
    junction at an angle; EXIT has the final answer. Call reset() after a turn
    manoeuvre, because sweeping the sensors across lines while spinning
    produces meaningless events.
    """

    def __init__(self, confirm_samples=2, clear_samples=3):
        if confirm_samples < 1 or clear_samples < 1:
            raise ValueError("sample counts must be >= 1")
        self.confirm_samples = confirm_samples
        self.clear_samples = clear_samples
        self.enter_count = 0
        self.exit_count = 0
        self.last_event = None
        self.reset()

    @classmethod
    def from_config(cls):
        import config
        return cls(confirm_samples=config.JUNCTION_CONFIRM_SAMPLES,
                   clear_samples=config.JUNCTION_CLEAR_SAMPLES)

    def reset(self, counts=False):
        """Forget any junction in progress (and the counters if counts=True)."""
        self.in_junction = False
        self.left = False
        self.right = False
        self._run = 0
        self._run_left = False
        self._run_right = False
        self._clear = 0
        self._enter_ms = 0
        if counts:
            self.enter_count = 0
            self.exit_count = 0
            self.last_event = None

    def update(self, bits, now_ms=None):
        """Feed one reading. Returns a JunctionEvent or None."""
        if now_ms is None:
            now_ms = ticks_ms()
        outer_left = bits & OUTER_LEFT
        outer_right = bits & OUTER_RIGHT
        inner = bits & INNER

        if not self.in_junction:
            left_hit = bool(outer_left and inner)
            right_hit = bool(outer_right and inner)
            if not (left_hit or right_hit):
                self._run = 0
                self._run_left = False
                self._run_right = False
                return None
            self._run += 1
            self._run_left = self._run_left or left_hit
            self._run_right = self._run_right or right_hit
            if self._run < self.confirm_samples:
                return None
            self.in_junction = True
            self.left = self._run_left
            self.right = self._run_right
            self._clear = 0
            self._enter_ms = now_ms
            self.enter_count += 1
            self.last_event = JunctionEvent(JunctionEvent.ENTER, self.left, self.right, t_ms=now_ms)
            return self.last_event

        if outer_left:
            self.left = True
        if outer_right:
            self.right = True
        if outer_left or outer_right:
            self._clear = 0
            return None
        self._clear += 1
        if self._clear < self.clear_samples:
            return None

        straight = bool(inner)
        event = JunctionEvent(JunctionEvent.EXIT, self.left, self.right, straight,
                              junction_shape(self.left, self.right, straight),
                              now_ms, ticks_diff(now_ms, self._enter_ms))
        self.exit_count += 1
        self.reset()
        self.last_event = event
        return event
