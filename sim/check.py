"""
Regression check: run this after every change to the robot code.

    python -m sim.check            (from the repository root, takes ~1 minute)

It checks the map (track_map.py) and then runs the robot's real main
program in the simulator in several situations, printing PASS / FAIL for
each. The map and the robot's speeds are estimates, so a PASS here means
"the logic works"; the real table still needs testing.

  map         every pad and rack slot can be reached from START and back;
              no node has two lines going the same way
  mission     the whole task: 4 boxes delivered, robot back at START,
              nothing hit
  full slots  boxes already in the first two slots for the first box: the
              robot looks into them, skips them, uses the third. The second
              box goes to the same rack from the other side: those three
              slots are full now (a slot is open on both sides), so it uses
              the fourth
  red light   the light is red when the robot reaches the ramp: it waits,
              then goes when it turns green
  poles       a south pole in the green pad: that box goes to the BOTTOM
              level (west line) instead of TOP (spine)
  stop button pressed again while driving: the robot stops at once
"""

import math
import sys


def _check_map():
    import track_map
    problems = []
    for node in track_map.PADS + track_map.SLOTS:
        for a, b in (("START", node), (node, "START")):
            try:
                track_map.find_route(a, b)
            except ValueError as e:
                problems.append(str(e))
    for a, b in track_map.LINES:
        if a not in track_map.NODES or b not in track_map.NODES:
            problems.append("line %s - %s uses an unknown node" % (a, b))
    return problems


def _printed(notes, text):
    return [line for t, x, y, line in notes if text in line]


def _scenario(name, check, **options):
    import os
    from sim.run import run
    os.makedirs("sim_output", exist_ok=True)
    png = os.path.join("sim_output", "check_%s.png" % name.replace(" ", "_"))
    world, notes, reason = run(png=png, quiet=True, **options)
    problems = check(world, notes, reason)
    return problems


def _mission_ok(world, notes, reason, boxes=4):
    problems = []
    done = _printed(notes, "mission complete")
    if not done:
        problems.append("mission did not complete (end: %s)" % reason.splitlines()[0])
    elif "%d boxes delivered" % boxes not in done[0]:
        problems.append(done[0])
    if world.events and any(kind == "stop" and "hit" in text for t, kind, text in world.events):
        problems.append("collision: %s" % world.stopped)
    import track_map
    sx, sy = track_map.NODES["START"]
    if done and math.hypot(world.x - sx, world.y - sy) > 0.06:
        problems.append("did not park on START: wheels at (%.2f, %.2f)" % (world.x, world.y))
    return problems


def main():
    results = []

    problems = _check_map()
    results.append(("map", problems))

    results.append(("mission", _scenario("mission", _mission_ok, seconds=400)))

    def full_slots(world, notes, reason):
        p = _mission_ok(world, notes, reason)
        skipped = [line.split()[0] for line in _printed(notes, "already has a box")]
        if skipped[:5] != ["ORANGE_E1", "ORANGE_E2", "ORANGE_W1", "ORANGE_W2", "ORANGE_W3"]:
            p.append("unexpected full slots: %s" % skipped)
        for slot in ("ORANGE_E3", "ORANGE_W4"):
            if not _printed(notes, "putting the box down in %s" % slot):
                p.append("no box was put into %s" % slot)
        return p
    results.append(("full slots", _scenario("full slots", full_slots, seconds=500,
                                            boxes=["ORANGE_E1", "ORANGE_E2"])))

    def red_light(world, notes, reason):
        p = _mission_ok(world, notes, reason)
        waits = [line for line in _printed(notes, "traffic light green after")]
        if not waits:
            p.append("never waited at the traffic light")
        elif int(waits[0].split("after")[1].split("ms")[0]) < 5000:
            p.append("did not wait for the red light: %s" % waits[0].strip())
        return p
    results.append(("red light", _scenario("red light", red_light, seconds=500, light=(40.0, 10.0, 0.0))))

    def poles(world, notes, reason):
        p = _mission_ok(world, notes, reason)
        if not _printed(notes, "putting the box down in ORANGE_W1"):
            p.append("south-pole box from PAD_GREEN did not go to ORANGE_W1")
        return p
    results.append(("poles", _scenario("poles", poles, seconds=400, poles={"PAD_GREEN": "south"})))

    def stop_button(world, notes, reason):
        p = []
        if not _printed(notes, "stopped by the start/stop button"):
            p.append("the second press did not stop the robot")
        moved_after = [r for r in world.trace if r[0] > 10.5 and (abs(r[5]) > 0.01 or abs(r[6]) > 0.01)]
        if moved_after:
            p.append("wheels still turning %.1f s after the stop" % (moved_after[-1][0] - 10.0))
        return p
    results.append(("stop button", _scenario("stop button", stop_button, seconds=15, presses=(0.3, 10.0))))

    print()
    failed = 0
    for name, problems in results:
        print("%-12s %s" % (name, "PASS" if not problems else "FAIL"))
        for problem in problems:
            print("             - %s" % problem)
        failed += bool(problems)
    print("\n%d of %d checks passed (pictures: sim_output/check_*.png)" % (len(results) - failed, len(results)))
    return failed


if __name__ == "__main__":
    sys.exit(1 if main() else 0)
