"""
Regression check: run this after every change to the robot code.

    python -m sim.check            (from the repository root, takes ~1 minute)

It checks MAP.py and then runs the robot's real main.py in the simulator in
several situations, printing PASS / FAIL for each. The simulated table and
the robot's speeds are estimates, so a PASS means "the logic works"; the
real table still needs testing.

  map          every pad and drop slot in MAP.py can be reached from START;
               no link is one-way (except the start box corners, which are
               never driven along)
  top level    north pole in GREEN_1 -> TOP level: the robot waits at the
               traffic light, then drops the box at LEFT_TOP_DROP_1
  bottom level south pole in GREEN_1 -> BOTTOM level: drop at
               LEFT_BOTTOM_DROP_1
  full slot    a box is already in LEFT_BOTTOM_DROP_1: the robot sees it,
               skips it and drops at LEFT_BOTTOM_DROP_2
  red light    the light is red when the robot reaches the ramp: it waits,
               then goes when it turns green
  pause/reset  a short press pauses (no wheel turns) and the next press
               continues; a long press stops and resets
  pick-up      ADDED (2026-10-27): the pick-up sequence on its own
               (tests/test_pickup.py, which runs control/pickup.py after
               the start press). The robot stands where the turn into the
               green pick-up box (PAD_GREEN) ends: rear axle on the junction
               W_PAD1, facing east along the stub. It must print
               "pickup: to_box", "pickup: grab", "pickup: u_turn",
               "pickup: back_to_main", "pickup: done" in this order (and no
               "pickup: failed"), be really close to the box when it grabs
               (front distance 60 mm or less), hold the box at the end and
               stop back at the side line facing west. Run for all four
               boxes. CHANGED (2026-10-27): the gripper servos must also
               stay inside config.SERVO_SAFE_RANGE the whole time.
  no grab sensor
               ADDED (2026-10-27): the same pick-up test with the grab
               distance sensor not started: it must print "pickup: failed"
               (no grab sensor) and the robot must not move at all.
  drop-off     ADDED (2026-10-27): the drop-off on its own
               (tests/test_dropoff.py -> control/dropoff.py
               release_and_push), presses at 0.3 / 4 / 8 s. The robot
               stands at the end of the rung to LEFT_BOTTOM_DROP_1 (rear
               axle 0.20 m behind ORANGE_W1), facing the rack (east). It
               must print the six "dropoff: ..." lines in order, back off /
               push / back out (each a real move, the push reaching past the
               point where the box was let go), not crash or hit the rack,
               end a little BEHIND its start (back + exit > push), still
               facing the rack, with servo2 open (top of its safe range).
               The servos must stay inside their safe ranges, except servo2
               down to config.SERVO_PUSH_ANGLE for the push.

Each run is checked up to the first box drop; driving out of a drop slot is
being reworked (custom drop-off function), so the runs stop there.
"""

import math
import os
import sys


def _check_map():
    from sim.run import _setup_paths
    _setup_paths()                  # functions.py imports motor.py, which needs (fake) machine
    import functions
    from MAP import MAP
    problems = []
    opposite = {"north": "south", "south": "north", "east": "west", "west": "east"}
    box_corners = ("TOP_LEFT", "TOP_RIGHT", "MIDDLE_LEFT", "MIDDLE_RIGHT")
    for name, node in MAP.items():
        for direction, other in node.items():
            if direction not in opposite or other is None:
                continue
            if other not in MAP:
                problems.append("%s.%s -> %s: no such node" % (name, direction, other))
            elif MAP[other].get(opposite[direction]) != name and not (name in box_corners and other in box_corners):
                problems.append("one-way link %s.%s -> %s" % (name, direction, other))
    robot = functions.Robot()
    for goal in MAP:
        if goal.startswith(("GREEN_", "RED_")) or ("_DROP_" in goal and not goal.startswith("JUNCTION_")):
            route = robot.find_route(goal)
            if not route or route[0] != goal:
                problems.append("no route from START to %s" % goal)
    return problems


def _printed(notes, text):
    return [(t, x, y, line) for t, x, y, line in notes if text in line]


def _on_rung(x, y, slot):
    """Is the rear axle (x, y) on the rung that leads to `slot` (a MAP.py drop
    name, e.g. LEFT_BOTTOM_DROP_1)?"""
    from sim import track_map
    from sim.arena import sim_name
    end = sim_name(slot)
    junction = list(track_map.NEIGHBOURS[end].values())[0]
    (jx, jy), (ex, ey) = track_map.NODES[junction], track_map.NODES[end]
    if abs(jy - ey) < 1e-6:                   # an east-west rung
        return abs(y - jy) < 0.06 and min(jx, ex) - 0.05 <= x <= max(jx, ex) + 0.05
    return abs(x - jx) < 0.06 and min(jy, ey) - 0.05 <= y <= max(jy, ey) + 0.05


def _scenario(name, check, **options):
    from sim.run import run
    os.makedirs("sim_output", exist_ok=True)
    png = os.path.join("sim_output", "check_%s.png" % name.replace(" ", "_").replace("/", "_"))
    world, notes, reason = run(png=png, quiet=True, **options)
    return check(world, notes, reason)


def _dropped_at(notes, slot):
    drops = _printed(notes, "DROPPING STUFF")
    if not drops:
        return ["no box was dropped"]
    t, x, y, line = drops[0]
    if not _on_rung(x, y, slot):
        return ["first box dropped at (%.2f, %.2f), not at %s" % (x, y, slot)]
    return []


# ADDED (2026-10-27): servo angles, read back from the PWM duty the robot
# code sent (actuators/servo.py: duty = MIN_DUTY + angle / MAX_ANGLE *
# (MAX_DUTY - MIN_DUTY), so angle = the same formula the other way round).
def _duty_to_angle(duty):
    from actuators.servo import Servo
    return (duty - Servo.MIN_DUTY) * Servo.MAX_ANGLE / float(Servo.MAX_DUTY - Servo.MIN_DUTY)


def _servo_angle(world, name):
    """The angle (degrees) last sent to servo `name` ("servo1" / "servo2"),
    or None if it never got pulses (or was switched off)."""
    import config
    duty = world.duty.get(config.SERVO_PINS[name])
    if not duty:
        return None
    return _duty_to_angle(duty)


def _servo_range_problems(world, push_allowed=False):
    """ADDED (2026-10-27): a problem for every gripper servo that was sent an
    angle outside config.SERVO_SAFE_RANGE at any time in the run.
    push_allowed=True: servo2 may also go down to config.SERVO_PUSH_ANGLE
    (the drop-off push, the only allowed exception)."""
    import config
    problems = []
    for name in sorted(config.SERVO_PINS):
        safe = config.SERVO_SAFE_RANGE.get(name)
        sent = world.duty_range.get(config.SERVO_PINS[name])   # (lowest, highest) duty
        if safe is None or sent is None:
            continue                    # range not measured, or the servo never moved
        low, high = safe
        if push_allowed and name in config.SERVO_PUSH_ANGLE:
            low = min(low, config.SERVO_PUSH_ANGLE[name])
        lowest, highest = _duty_to_angle(sent[0]), _duty_to_angle(sent[1])
        # 0.5 degree margin: Servo rounds the duty down to a whole number
        if lowest < low - 0.5 or highest > high + 0.5:
            problems.append("%s was sent %.1f - %.1f deg, outside its allowed %d - %d deg"
                            % (name, lowest, highest, low, high))
    return problems


def _phase_order_problems(notes, prefix, names):
    """ADDED (2026-10-27): problems if a line starting with `prefix` + name
    was never printed, or printed before the one of the name before it.
    `names` lists them in the order they must appear."""
    problems = []
    last_t, last_name = -1.0, None
    for name in names:
        lines = _printed(notes, prefix + name)
        if not lines:
            problems.append("never printed \"%s%s\"" % (prefix, name))
            continue
        t = lines[0][0]
        if t < last_t:
            problems.append("\"%s%s\" (%.2f s) printed before \"%s%s\" (%.2f s)"
                            % (prefix, name, t, prefix, last_name, last_t))
        last_t, last_name = t, name
    return problems


# ADDED (2026-10-27): the "pick-up" scenario (see the top of this file).
# CHANGED (2026-10-27): run for every box, not only GREEN_1 (each box's
# U-turn spins its own way, config.PICKUP_UTURN_DIRECTION).
PICKUP_ENTRY = "tests.test_pickup:test_pickup"
PICKUP_PHASES = ("to_box", "grab", "u_turn", "back_to_main", "done")   # must be printed in this order
PICKUP_BOXES = ("GREEN_1", "GREEN_2", "RED_1", "RED_2")                 # MAP.py names of the four boxes


def check_pick_up(entry=PICKUP_ENTRY, seconds=40, box="GREEN_1", **options):
    """Run the pick-up sequence from the end of the turn into `box` (a MAP.py
    name, e.g. "GREEN_1") and return a list of problems (empty = PASS).

    entry: the module:function to run (the default is the calibration test
    tests/test_pickup.py; another entry can be given to try a stand-in).
    options: passed on to sim.run.run (e.g. kwargs={...} for the entry)."""
    if entry == PICKUP_ENTRY:
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        if not os.path.exists(os.path.join(root, "tests", "test_pickup.py")):
            return ["tests/test_pickup.py missing"]

    from sim import track_map
    from sim.arena import sim_name
    pad = sim_name(box)                          # e.g. GREEN_1 -> PAD_GREEN
    junction = track_map.pad_junction(pad)       # e.g. W_PAD1
    (jx, jy), (px, py) = track_map.NODES[junction], track_map.NODES[pad]
    heading_in = math.degrees(math.atan2(py - jy, px - jx)) % 360   # facing the box
    heading_back = (heading_in + 180.0) % 360                        # after the U-turn
    if entry == PICKUP_ENTRY and "kwargs" not in options:
        options["kwargs"] = {"box": box}

    def off_by(a, b):
        """Smallest difference between two headings (degrees)."""
        return abs((a - b + 180.0) % 360 - 180.0)

    # The world's TRUE front distance (not the robot's reading) at the
    # moment the robot prints "pickup: grab ...": recorded by run()'s
    # on_print hook. (Later the box is taken and the distance changes.)
    grab_distance = []
    # The robot's heading when the U-turn ends ("pickup: back_to_main"), for
    # a clearer message when the spin stops on the wrong line.
    u_turn_end_heading = []

    def on_print(world, line):
        if "pickup: grab" in line and not grab_distance:
            grab_distance.append(world.front_distance_mm())
        if "pickup: back_to_main" in line and not u_turn_end_heading:
            u_turn_end_heading.append(math.degrees(world.th) % 360)

    def check(world, notes, reason):
        from sim import track_map
        problems = []
        if reason.startswith("robot code crashed"):
            problems.append("the robot code crashed: %s" % reason.strip().splitlines()[-1])
        failed = _printed(notes, "pickup: failed")
        if failed:
            problems.append("printed %r" % failed[0][3])
        # every phase printed, and in the right order
        # (CHANGED 2026-10-27: the loop moved to _phase_order_problems, shared with the drop-off)
        problems += _phase_order_problems(notes, "pickup: ", PICKUP_PHASES)
        # how the run ended, when the sequence did not finish
        if not _printed(notes, "pickup: done") and not reason.startswith("robot code crashed"):
            problems.append("the run ended with: %s" % world.stopped)
        # the U-turn must end facing back along the stub (west), not on
        # another line (e.g. the other pad stub, only 0.17 m away)
        if u_turn_end_heading and off_by(u_turn_end_heading[0], heading_back) > 35.0:
            problems.append("the U-turn ended facing %.0f deg, not about %.0f: it stopped on "
                            "another line" % (u_turn_end_heading[0], heading_back))
        # close enough to the box when it grabbed
        if grab_distance:
            d = grab_distance[0]
            if d is None or d > 60:
                problems.append("at \"pickup: grab\" the box was %s ahead (should be 60 mm or less)"
                                % ("nothing" if d is None else "%d mm" % d))
        # holding the box at the end
        if world.carrying != pad:
            problems.append("not holding the %s box at the end (carrying: %s)" % (pad, world.carrying))
        # back at the side line, facing west (back along the stub)
        off = math.hypot(world.x - jx, world.y - jy)
        if off > 0.30:
            problems.append("ended with the rear axle %.2f m from %s (should be 0.30 m or less)"
                            % (off, junction))
        heading = math.degrees(world.th) % 360
        if off_by(heading, heading_back) > 35.0:
            problems.append("ended facing %.0f deg (should be about %.0f)" % (heading, heading_back))
        # ADDED (2026-10-27): the gripper servos never left their safe ranges
        problems += _servo_range_problems(world)
        return problems

    try:
        return _scenario("pick-up " + box, check, entry=entry, seconds=seconds, at=junction,
                         heading=heading_in, back=0.0, on_print=on_print, **options)
    except Exception as e:      # never let one scenario stop the whole check
        return ["the simulation itself failed: %s: %s" % (type(e).__name__, e)]


def check_pick_up_no_grab_sensor(box="GREEN_1"):
    """ADDED (2026-10-27): the pick-up test (same start as check_pick_up)
    with the grab distance sensor not started. The pick-up must fail at
    once because of the missing sensor, and the robot must not move at all.
    Returns a list of problems (empty = PASS)."""
    from sim import track_map
    from sim.arena import sim_name
    pad = sim_name(box)                          # e.g. GREEN_1 -> PAD_GREEN
    junction = track_map.pad_junction(pad)       # e.g. W_PAD1
    (jx, jy), (px, py) = track_map.NODES[junction], track_map.NODES[pad]
    heading_in = math.degrees(math.atan2(py - jy, px - jx)) % 360   # facing the box

    def check(world, notes, reason):
        problems = []
        if reason != "robot code returned":
            problems.append("the run ended with: %s" % reason.strip().splitlines()[-1])
        failed = _printed(notes, "pickup: failed")
        if not failed:
            problems.append("never printed \"pickup: failed\"")
        elif "grab distance sensor" not in failed[0][3]:
            problems.append("failed, but not because of the missing grab sensor: %r" % failed[0][3])
        if _printed(notes, "pickup: to_box"):
            problems.append("went on to \"pickup: to_box\" without a grab sensor")
        if world.distance > 0.002:
            problems.append("the robot moved %.3f m (it must not move at all)" % world.distance)
        return problems

    try:
        return _scenario("pick-up no grab sensor", check, entry=PICKUP_ENTRY, seconds=20, at=junction,
                         heading=heading_in, back=0.0, kwargs={"box": box}, grab_sensor=False)
    except Exception as e:      # never let one scenario stop the whole check
        return ["the simulation itself failed: %s: %s" % (type(e).__name__, e)]


# ADDED (2026-10-27): the "drop-off" scenario (see the top of this file).
DROPOFF_ENTRY = "tests.test_dropoff:test_dropoff"
DROPOFF_SLOT = "LEFT_BOTTOM_DROP_1"       # MAP.py name of the slot (simulator: ORANGE_W1)
DROPOFF_PRESSES = (0.3, 4.0, 8.0)         # open the gripper / close it on the box / run the drop-off
# the steps control/dropoff.py prints ("dropoff: <step> ..."), in this order
DROPOFF_STEPS = ("released", "backed off", "gripper closed for the push", "pushed", "backed out", "done")


def check_drop_off(entry=DROPOFF_ENTRY, seconds=20, slot=DROPOFF_SLOT, **options):
    """Run the drop-off with the robot at the end of the rung to `slot` (a
    MAP.py drop name), facing the rack, and return a list of problems
    (empty = PASS). entry / options: as for check_pick_up."""
    if entry == DROPOFF_ENTRY:
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        if not os.path.exists(os.path.join(root, "tests", "test_dropoff.py")):
            return ["tests/test_dropoff.py missing"]

    from sim import track_map
    from sim.arena import sim_name
    end = sim_name(slot)                                       # e.g. ORANGE_W1 (the rung end)
    junction = list(track_map.NEIGHBOURS[end].values())[0]     # e.g. W1 (where the rung starts)
    (jx, jy), (ex, ey) = track_map.NODES[junction], track_map.NODES[end]
    heading = math.atan2(ey - jy, ex - jx)          # from the junction towards the rack (radians)
    back = 0.20                                     # rear axle this far behind the rung end
    x0, y0 = ex - back * math.cos(heading), ey - back * math.sin(heading)   # start of the rear axle

    def ahead(x, y):
        """How far (m) the point (x, y) is in front of the start position,
        measured along the starting heading (negative = behind it)."""
        return (x - x0) * math.cos(heading) + (y - y0) * math.sin(heading)

    # For each step: (how far ahead of the start the robot was, servo2
    # angle) at the moment its "dropoff: <step>" line was printed.
    at_step = {}

    def on_print(world, line):
        for step in DROPOFF_STEPS:
            if line.startswith("dropoff: " + step) and step not in at_step:
                at_step[step] = (ahead(world.x, world.y), _servo_angle(world, "servo2"))

    def check(world, notes, reason):
        import config
        problems = []
        if reason.startswith("robot code crashed"):
            problems.append("the robot code crashed: %s" % reason.strip().splitlines()[-1])
        elif reason != "robot code returned":
            problems.append("the run ended with: %s" % reason)    # e.g. hit the rack, time limit
        problems += _phase_order_problems(notes, "dropoff: ", DROPOFF_STEPS)

        # each move really happened, in the right direction (positions in m)
        if len(at_step) == len(DROPOFF_STEPS):
            released = at_step["released"][0]
            backed_off = at_step["backed off"][0]
            closed, closed_angle = at_step["gripper closed for the push"]
            pushed = at_step["pushed"][0]
            backed_out = at_step["backed out"][0]
            if backed_off > released - 0.005:
                problems.append("did not back off after letting go (moved %+.3f m)" % (backed_off - released))
            push_angle = config.SERVO_PUSH_ANGLE.get("servo2", config.SERVO_SAFE_RANGE["servo2"][0])
            if closed_angle is None or abs(closed_angle - push_angle) > 1.0:
                problems.append("servo2 at %s deg for the push, not SERVO_PUSH_ANGLE %s"
                                % ("no pulses" if closed_angle is None else "%.0f" % closed_angle, push_angle))
            if pushed < closed + 0.01:
                problems.append("did not drive forward for the push (moved %+.3f m)" % (pushed - closed))
            if pushed <= released:
                problems.append("the push stopped %.3f m short of where the box was let go: the box is not "
                                "pushed in (DROP_PUSH_MS must be longer than DROP_BACK_MS)" % (released - pushed))
            if backed_out > pushed - 0.01:
                problems.append("did not back out after the push (moved %+.3f m)" % (backed_out - pushed))

        # in the end a little BEHIND the start (back + exit > push)
        moved = ahead(world.x, world.y)
        if not -0.15 <= moved <= -0.005:
            problems.append("ended %+.3f m from the start along the heading (should be a little behind it: "
                            "-0.005 to -0.150 m)" % moved)
        # all moves are straight: still facing the rack
        turned = abs((math.degrees(world.th - heading) + 180.0) % 360 - 180.0)
        if turned > 5.0:
            problems.append("ended turned by %.0f deg (all drop-off moves are straight)" % turned)
        # servo2 open again (top of its safe range), ready for the next pick-up
        open_angle = config.SERVO_SAFE_RANGE["servo2"][1]
        angle = _servo_angle(world, "servo2")
        if angle is None or abs(angle - open_angle) > 1.0:
            problems.append("servo2 ended at %s, not open (%d deg)"
                            % ("no pulses" if angle is None else "%.0f deg" % angle, open_angle))
        # the servos never left their safe ranges (servo2: down to the push angle)
        problems += _servo_range_problems(world, push_allowed=True)
        return problems

    options.setdefault("presses", DROPOFF_PRESSES)
    try:
        return _scenario("drop-off", check, entry=entry, seconds=seconds, at=end,
                         heading=math.degrees(heading), back=back, on_print=on_print, **options)
    except Exception as e:      # never let one scenario stop the whole check
        return ["the simulation itself failed: %s: %s" % (type(e).__name__, e)]


def main():
    results = [("map", _check_map())]

    def top_level(world, notes, reason):
        p = _dropped_at(notes, "LEFT_TOP_DROP_1")
        if not _printed(notes, "waiting for the traffic light"):
            p.append("did not stop at the traffic light")
        return p
    results.append(("top level", _scenario("top level", top_level, seconds=70)))

    results.append(("bottom level", _scenario(
        "bottom level", lambda w, n, r: _dropped_at(n, "LEFT_BOTTOM_DROP_1"),
        seconds=70, poles={"GREEN_1": "south"})))

    def full_slot(world, notes, reason):
        p = _dropped_at(notes, "LEFT_BOTTOM_DROP_2")
        if not _printed(notes, "LEFT_BOTTOM_DROP_1 already has a box"):
            p.append("did not see the box in LEFT_BOTTOM_DROP_1")
        return p
    results.append(("full slot", _scenario("full slot", full_slot, seconds=90,
                                           poles={"GREEN_1": "south"}, boxes=["LEFT_BOTTOM_DROP_1"])))

    def red_light(world, notes, reason):
        p = _dropped_at(notes, "LEFT_TOP_DROP_1")
        waits = _printed(notes, "traffic light green - go")
        if not waits:
            p.append("never went on from the traffic light")
        elif int(waits[0][3].split("waited")[1].split("ms")[0]) < 5000:
            p.append("did not wait for the red light: %s" % waits[0][3])
        return p
    results.append(("red light", _scenario("red light", red_light, seconds=90, light=(45.0, 10.0, 0.0))))

    def pause_reset(world, notes, reason):
        p = []
        if not _printed(notes, "PAUSED"):
            p.append("the short press did not pause")
        if not _printed(notes, "continuing after"):
            p.append("the next press did not continue")
        if not _printed(notes, "STOP AND RESET"):
            p.append("the long press did not stop and reset")
        moving = [r for r in world.trace if 10.6 < r[0] < 14.0 and (abs(r[5]) > 0.01 or abs(r[6]) > 0.01)]
        if moving:
            p.append("wheels turning during the pause (t = %.1f s)" % moving[0][0])
        return p
    results.append(("pause/reset", _scenario("pause reset", pause_reset, seconds=30,
                                             presses=(0.3, 10.0, 14.0, 20.0), press_hold={20.0: 2000})))

    # ADDED (2026-10-27): the pick-up sequence on its own (see check_pick_up)
    for box in PICKUP_BOXES:
        results.append(("pick-up " + box, check_pick_up(box=box)))
    # ADDED (2026-10-27): no grab sensor -> no movement; the drop-off on its own
    results.append(("no grab sensor", check_pick_up_no_grab_sensor()))
    results.append(("drop-off", check_drop_off()))

    print()
    failed = 0
    for name, problems in results:
        print("%-13s %s" % (name, "PASS" if not problems else "FAIL"))
        for problem in problems:
            print("              - %s" % problem)
        failed += bool(problems)
    print("\n%d of %d checks passed (pictures: sim_output/check_*.png)" % (len(results) - failed, len(results)))
    return failed


if __name__ == "__main__":
    sys.exit(1 if main() else 0)
