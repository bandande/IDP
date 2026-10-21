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
