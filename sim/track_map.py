"""
The SIMULATOR's map of the competition table: where the white lines go,
with positions in metres. The robot does not use this file - its map is
MAP.py (node names and neighbours only). The OLD NAMES table below shows
which MAP.py node is which node here.

HOW THE MAP WORKS
  * Every place where the robot can stop, turn or do something is a NODE
    with a name and a position (x, y) in metres.
  * LINES lists which nodes are joined by a straight white line.
  * Everything else is worked out from these two lists when the program
    starts: which node is next to which, and in which compass direction
    (north / east / south / west). So the map can never contain a "one-way"
    link or a wrong direction - if you move a node, its directions update.

DIRECTIONS
  x grows to the EAST, y grows to the NORTH. The robot starts in the start
  box on START, facing NORTH (towards the racks). Origin = the south-west
  corner of the table.

THE TABLE (not to scale; [O] orange rack, [P] purple rack, = ramp)

      W_END         S_END         E_END        T-bars at the north ends
        |             |             |
     W6 +--   [O]  --+ S6 +--  [P]  --+ E6
        :             :              :         6 rungs on each line; every
     W1 +--   [O]  --+ S1 +--  [P]  --+ E1      rung ends at a rack slot
        |             =              |
  W_PAD2+--(red)      =    (blue)--+ E_PAD2
  W_PAD1+--(green)    =  (yellow)--+ E_PAD1
        |       BOX_NW+-BOX_N-+BOX_NE|
        |             |   |   |      |
  CORNER_W------BOX_W-+-START-+-BOX_E------CORNER_E    start line
                      |       |
                BOX_SW+-------+BOX_SE                  start box

  * A rack slot can be reached from both sides of its rack. Slot names say
    which rack, which side the robot comes from, and the slot number
    (1 = southmost):  ORANGE_W3 = orange rack, slot 3, from the west line;
    ORANGE_E3 = the same slot from the spine; PURPLE_W3 from the spine;
    PURPLE_E3 from the east line.
  * The start box is drawn round START; its edges cross the start line and
    the spine, so the robot sees them as junctions (BOX_W, BOX_E, BOX_N).

MEASUREMENTS
  The positions below are estimates from a video of the table. Only the
  short distances matter to the robot (see navigation.py: a node closer
  than LINE_SENSOR_AHEAD_M to the previous one is already behind the line
  sensors after a turn). TODO: measure the real table, especially the
  distance between the two pad stubs on each side line (PAD_Y; about
  17 cm in the video).

OLD NAMES (MAP.py, before 2026-10-20) -> NEW NAMES
  START -> START          TOP_CENTRE -> BOX_N       MIDDLE_LEFT -> BOX_W
  JUNCTION_LEFT_BOTTOM -> CORNER_W                  MIDDLE_RIGHT -> BOX_E
  JUNCTION_GREEN_1 / GREEN_1 -> W_PAD1 / PAD_GREEN
  JUNCTION_GREEN_2 / GREEN_2 -> W_PAD2 / PAD_RED
  JUNCTION_RED_1 / RED_1     -> E_PAD1 / PAD_YELLOW
  JUNCTION_RED_2 / RED_2     -> E_PAD2 / PAD_BLUE
  JUNCTION_LEFT_BOTTOM_DROP_k / LEFT_BOTTOM_DROP_k   -> Wk / ORANGE_Wk
  JUNCTION_TOP_DROP_k / LEFT_TOP_DROP_k / RIGHT_TOP_DROP_k -> Sk / ORANGE_Ek / PURPLE_Wk
  JUNCTION_RIGHT_BOTTOM_DROP_k / RIGHT_BOTTOM_DROP_k -> Ek / PURPLE_Ek
"""

# ---------------------------------------------------------------------------
# Measurements (metres). TODO: measure the real table.
# ---------------------------------------------------------------------------
TABLE_WIDTH = 1.96        # east-west
TABLE_LENGTH = 2.40       # south-north
X_WEST_LINE = 0.30        # the long line along the west side
X_SPINE = 0.98            # the long line from START, over the ramp, between the racks
X_EAST_LINE = 1.66        # the long line along the east side
Y_START_LINE = 0.35       # the start line (west corner - START - east corner)
BOX_HALF_WIDTH = 0.15     # start box: half of its east-west size
BOX_NORTH = 0.08          # start box: north edge is this far north of the start line
BOX_SOUTH = 0.22          # start box: south edge is this far south of the start line
PAD_Y = (0.58, 0.75)      # the two pad stubs on each side line (southern one first)
PAD_DISTANCE = 0.15       # from a side line to the centre of its pads
RUNG_Y = (1.12, 1.24, 1.36, 1.48, 1.60, 1.72)   # the six rungs, south to north
RUNG_LENGTH = 0.25        # from a long line to the end of a rung (just short of the rack)
Y_END = 2.25              # T-bars at the north ends of the three long lines

# ---------------------------------------------------------------------------
# Nodes: name -> (x, y)
# ---------------------------------------------------------------------------
NODES = {
    # start line and start box
    "START": (X_SPINE, Y_START_LINE),
    "CORNER_W": (X_WEST_LINE, Y_START_LINE),
    "CORNER_E": (X_EAST_LINE, Y_START_LINE),
    "BOX_W": (X_SPINE - BOX_HALF_WIDTH, Y_START_LINE),
    "BOX_E": (X_SPINE + BOX_HALF_WIDTH, Y_START_LINE),
    "BOX_N": (X_SPINE, Y_START_LINE + BOX_NORTH),
    "BOX_NW": (X_SPINE - BOX_HALF_WIDTH, Y_START_LINE + BOX_NORTH),
    "BOX_NE": (X_SPINE + BOX_HALF_WIDTH, Y_START_LINE + BOX_NORTH),
    "BOX_SW": (X_SPINE - BOX_HALF_WIDTH, Y_START_LINE - BOX_SOUTH),
    "BOX_SE": (X_SPINE + BOX_HALF_WIDTH, Y_START_LINE - BOX_SOUTH),
    # pick-up pads and the junctions of their stubs
    "W_PAD1": (X_WEST_LINE, PAD_Y[0]),
    "W_PAD2": (X_WEST_LINE, PAD_Y[1]),
    "E_PAD1": (X_EAST_LINE, PAD_Y[0]),
    "E_PAD2": (X_EAST_LINE, PAD_Y[1]),
    "PAD_GREEN": (X_WEST_LINE + PAD_DISTANCE, PAD_Y[0]),
    "PAD_RED": (X_WEST_LINE + PAD_DISTANCE, PAD_Y[1]),
    "PAD_YELLOW": (X_EAST_LINE - PAD_DISTANCE, PAD_Y[0]),
    "PAD_BLUE": (X_EAST_LINE - PAD_DISTANCE, PAD_Y[1]),
    # north ends (T-bars)
    "W_END": (X_WEST_LINE, Y_END),
    "S_END": (X_SPINE, Y_END),
    "E_END": (X_EAST_LINE, Y_END),
}
for _k, _y in enumerate(RUNG_Y, 1):
    NODES["W%d" % _k] = (X_WEST_LINE, _y)                          # west line junction
    NODES["S%d" % _k] = (X_SPINE, _y)                              # spine crossing
    NODES["E%d" % _k] = (X_EAST_LINE, _y)                          # east line junction
    NODES["ORANGE_W%d" % _k] = (X_WEST_LINE + RUNG_LENGTH, _y)     # orange slot, from the west line
    NODES["ORANGE_E%d" % _k] = (X_SPINE - RUNG_LENGTH, _y)         # orange slot, from the spine
    NODES["PURPLE_W%d" % _k] = (X_SPINE + RUNG_LENGTH, _y)         # purple slot, from the spine
    NODES["PURPLE_E%d" % _k] = (X_EAST_LINE - RUNG_LENGTH, _y)     # purple slot, from the east line

PADS = ("PAD_GREEN", "PAD_RED", "PAD_YELLOW", "PAD_BLUE")
# The corners of the start box outline: the robot sees the box lines but never
# drives along them, so routes do not go through these nodes.
NOT_FOR_DRIVING = ("BOX_NW", "BOX_NE", "BOX_SW", "BOX_SE")
SLOTS = tuple(name for name in NODES if name.startswith("ORANGE_") or name.startswith("PURPLE_"))

# ---------------------------------------------------------------------------
# Lines: pairs of nodes joined by a straight white line
# ---------------------------------------------------------------------------
LINES = [
    # start line, through the start box
    ("CORNER_W", "BOX_W"), ("BOX_W", "START"), ("START", "BOX_E"), ("BOX_E", "CORNER_E"),
    # start box outline (its north edge crosses the spine at BOX_N)
    ("BOX_NW", "BOX_N"), ("BOX_N", "BOX_NE"),
    ("BOX_NW", "BOX_W"), ("BOX_W", "BOX_SW"), ("BOX_SW", "BOX_SE"),
    ("BOX_SE", "BOX_E"), ("BOX_E", "BOX_NE"),
    # spine: START -> box edge -> ramp -> S1 ... S6 -> T-bar
    ("START", "BOX_N"), ("BOX_N", "S1"),
    # west line: corner -> pads -> W1 ... W6 -> T-bar
    ("CORNER_W", "W_PAD1"), ("W_PAD1", "W_PAD2"), ("W_PAD2", "W1"),
    ("W_PAD1", "PAD_GREEN"), ("W_PAD2", "PAD_RED"),
    # east line: corner -> pads -> E1 ... E6 -> T-bar
    ("CORNER_E", "E_PAD1"), ("E_PAD1", "E_PAD2"), ("E_PAD2", "E1"),
    ("E_PAD1", "PAD_YELLOW"), ("E_PAD2", "PAD_BLUE"),
    ("W6", "W_END"), ("S6", "S_END"), ("E6", "E_END"),
]
for _k in range(1, len(RUNG_Y)):
    LINES += [("W%d" % _k, "W%d" % (_k + 1)), ("S%d" % _k, "S%d" % (_k + 1)),
              ("E%d" % _k, "E%d" % (_k + 1))]
for _k in range(1, len(RUNG_Y) + 1):
    LINES += [("W%d" % _k, "ORANGE_W%d" % _k), ("S%d" % _k, "ORANGE_E%d" % _k),
              ("S%d" % _k, "PURPLE_W%d" % _k), ("E%d" % _k, "PURPLE_E%d" % _k)]

# ---------------------------------------------------------------------------
# Worked out from NODES and LINES (do not edit)
# ---------------------------------------------------------------------------
COMPASS = ("north", "east", "south", "west")    # clockwise order


def direction(from_node, to_node):
    """Compass direction of the straight line from one node to another."""
    (x1, y1), (x2, y2) = NODES[from_node], NODES[to_node]
    dx, dy = x2 - x1, y2 - y1
    if abs(dx) > abs(dy):
        return "east" if dx > 0 else "west"
    return "north" if dy > 0 else "south"


def _build_neighbours():
    neighbours = {name: {} for name in NODES}
    for a, b in LINES:
        for here, there in ((a, b), (b, a)):
            d = direction(here, there)
            if d in neighbours[here]:
                raise ValueError("map: %s has two lines going %s (%s and %s)"
                                 % (here, d, neighbours[here][d], there))
            neighbours[here][d] = there
    return neighbours


NEIGHBOURS = _build_neighbours()     # name -> {"north": name, ...}


def distance(a, b):
    """Straight-line distance between two nodes in metres."""
    (x1, y1), (x2, y2) = NODES[a], NODES[b]
    return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5


def is_dead_end(node):
    """True for nodes with only one line: pads, rack slots, T-bar ends."""
    return len(NEIGHBOURS[node]) == 1


def turn_needed(heading, new_direction):
    """Which way to turn when facing `heading` to face `new_direction`:
    "straight", "right", "about" (180 degrees) or "left"."""
    steps = (COMPASS.index(new_direction) - COMPASS.index(heading)) % 4
    return ("straight", "right", "about", "left")[steps]


def turn_result(heading, turn):
    """The heading after turning `turn` while facing `heading`."""
    steps = {"straight": 0, "right": 1, "about": 2, "left": 3}[turn]
    return COMPASS[(COMPASS.index(heading) + steps) % 4]


def side_branches(node, heading):
    """For a robot arriving at `node` facing `heading`: is there a line going
    off to its left, and one to its right? Returns (left, right)."""
    left = turn_result(heading, "left") in NEIGHBOURS[node]
    right = turn_result(heading, "right") in NEIGHBOURS[node]
    return left, right


def find_route(start, goal):
    """Shortest route (fewest nodes) as a list [start, ..., goal].

    Breadth-first search: look at all nodes 1 step away, then 2 steps, ...
    until the goal is found. Dead ends (pads, slots, T-bars) are never passed
    through, only used as the start or the goal, and the start box corners
    (NOT_FOR_DRIVING) are never used. Raises ValueError if there is no route."""
    if start not in NODES or goal not in NODES:
        raise ValueError("unknown node: %s or %s" % (start, goal))
    came_from = {start: None}
    frontier = [start]
    while frontier:
        next_frontier = []
        for node in frontier:
            if node == goal:
                route = []
                while node is not None:
                    route.append(node)
                    node = came_from[node]
                route.reverse()
                return route
            if node != start and is_dead_end(node):
                continue
            for neighbour in NEIGHBOURS[node].values():
                if neighbour in NOT_FOR_DRIVING and neighbour != goal:
                    continue
                if neighbour not in came_from:
                    came_from[neighbour] = node
                    next_frontier.append(neighbour)
        frontier = next_frontier
    raise ValueError("no route from %s to %s" % (start, goal))
