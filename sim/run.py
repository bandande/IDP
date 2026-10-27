"""
Run the robot code on the PC against the simulated table.

    python -m sim.run                              # main.py's main(), 240 s
    python -m sim.run --time 120 --png run.png
    python -m sim.run --boxes ORANGE_W1 --poles GREEN=south --light 8,8,4
    python -m sim.run --spec sensor_ahead=0.18 track_width=0.25 vmax=0.18
    python -m sim.run tests.test_line_follow:test_line_follow --kwargs "{\"plan\": [\"L\", \"S\"]}"

The robot code runs UNCHANGED: sim/fakes is put first on sys.path, so its
`import machine` / `import utime` get the simulator's versions, which talk
to the simulated world (sim/world.py). Pins come from config.py, the table
from sim/track_map.py. The start/stop button is pressed automatically at
--press seconds (default 0.3 s; several allowed; --press none = never).

The task-sensor world:
  --poles   magnet pole in each pick-up pad (default GREEN/YELLOW north,
            RED/BLUE south), e.g. --poles GREEN=south
  --boxes   rack slots that already hold a box (the front distance sensor
            sees them), e.g. --boxes ORANGE_W1 PURPLE_E2
  --light   traffic light cycle red_s,green_s[,phase_s] (default 8,8: red
            for the first 8 s, then green for 8 s, ...). It sends IR while
            green, as on the real table. --light none = no light at all
            (no IR ever, so the robot waits at the ramp forever).
  Names may be sim/track_map.py names, pad colours (GREEN, ...) or the
  MAP.py names (LEFT_BOTTOM_DROP_1, GREEN_1, ...).

Output: the robot's printed lines (prefixed with the simulated time), a
summary, and a picture of the path (--png, default sim_run.png).
"""

import argparse
import io
import json
import math
import os
import shutil
import sys
import tempfile
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
FAKES = os.path.join(HERE, "fakes")

HEADINGS = {"east": 0.0, "north": 90.0, "west": 180.0, "south": 270.0}
DEFAULT_POLES = {"PAD_GREEN": "north", "PAD_RED": "south", "PAD_YELLOW": "north", "PAD_BLUE": "south"}


class _TimedOutput(io.TextIOBase):
    """stdout replacement: prefixes each printed line with the simulated time
    and remembers it (with the robot position) for the picture."""

    def __init__(self, world, real, quiet=False, on_print=None):
        self.world = world
        self.real = real
        self.quiet = quiet
        self.notes = []
        self._buf = ""
        # ADDED (2026-10-27): on_print(world, line) is called for every
        # printed line, at the simulated moment it is printed (sim/check.py
        # uses it to read the world's true state at a given print, e.g. the
        # real front distance when the pick-up prints "pickup: grab").
        self.on_print = on_print

    def write(self, s):
        self._buf += s
        while "\n" in self._buf:
            line, self._buf = self._buf.split("\n", 1)
            w = self.world
            self.notes.append((w.t(), w.x, w.y, line))
            if not self.quiet:
                self.real.write("[%7.3f] %s\n" % (w.t(), line))
            if self.on_print is not None:      # ADDED (2026-10-27), see __init__
                self.on_print(w, line)
        return len(s)

    def flush(self):
        self.real.flush()


def _setup_paths():
    """Make `import machine` / `import utime` load the simulator's fakes."""
    for p in (ROOT, FAKES):
        if p in sys.path:
            sys.path.remove(p)
    sys.path.insert(0, ROOT)
    sys.path.insert(0, FAKES)
    import machine
    import utime
    if not os.path.abspath(machine.__file__).startswith(FAKES) or \
            not os.path.abspath(utime.__file__).startswith(FAKES):
        raise RuntimeError("could not load the simulator's fake machine/utime")


def _forget_robot_modules():
    """Remove the robot's modules from sys.modules, so every run starts from
    freshly imported code (the robot code keeps state in its modules, e.g.
    the MAP dict that main.py marks COLLECTED / FILLED)."""
    for name, module in list(sys.modules.items()):
        path = getattr(module, "__file__", None) or ""
        path = os.path.abspath(path) if path else ""
        if path.startswith(ROOT) and not path.startswith(HERE):
            del sys.modules[name]


def _parse_spec(items):
    out = {}
    for item in items or ():
        k, v = item.split("=", 1)
        v = v.strip()
        out[k.strip()] = tuple(float(x) for x in v.split(",")) if "," in v else float(v)
    return out


def _parse_light(text):
    if text is None or text.lower() == "none":
        return None
    parts = [float(v) for v in text.split(",")]
    return (parts[0], parts[1], parts[2] if len(parts) > 2 else 0.0)


def _grab_sensor_missing():
    """ADDED (2026-10-27): stand-in for task_sensors.make_grab_tof when the
    grab sensor is switched off (run(..., grab_sensor=False)): it fails the
    same way a real VL53L0X that does not answer does."""
    raise OSError("simulator: grab distance sensor switched off")


def make_world(track, spec, pose, t_end, stop_on_collision=True):
    import config
    from sim.world import World
    released = 1 if config.CRASH_ACTIVE_LOW else 0   # crash sensor level when not pressed
    pins = {
        "line": [config.LINE_OUTER_LEFT_PIN, config.LINE_INNER_LEFT_PIN,
                 config.LINE_INNER_RIGHT_PIN, config.LINE_OUTER_RIGHT_PIN],
        "line_white_level": 1 if config.LINE_WHITE_IS_HIGH else 0,
        "left_dir": config.MOTOR_LEFT_DIR_PIN, "left_pwm": config.MOTOR_LEFT_PWM_PIN,
        "right_dir": config.MOTOR_RIGHT_DIR_PIN, "right_pwm": config.MOTOR_RIGHT_PWM_PIN,
        "forward_level": 0,
        "left_invert": config.MOTOR_LEFT_INVERT, "right_invert": config.MOTOR_RIGHT_INVERT,
        # CHANGED (2026-10-23): the push button (PUSH_BUTTON_PIN) is a simulated
        # button too, wired the same way as the crash sensors.
        "crash": {config.CRASH_LEFT_PIN: released, config.CRASH_RIGHT_PIN: released,
                  config.PUSH_BUTTON_PIN: released},
    }
    return World(track, spec, pins, pose, t_end, stop_on_collision=stop_on_collision)


def run(entry="main:main", seconds=240.0, presses=(0.3,), at="START", heading="north", back=0.20,
        pose=None, spec_overrides=None, kwargs=None, png="sim_run.png", quiet=False,
        stop_on_collision=True, pad_fill_white=True, boxes=(), poles=None, light=(8.0, 8.0, 0.0),
        press_hold=None, on_print=None, grab_sensor=True):
    """Run one simulation. Returns (world, notes, end_reason).

    notes: [(t, x, y, printed line)]; end_reason: why the run ended.
    press_hold: {press time: hold ms} for presses longer than the default 80 ms.
    on_print: ADDED (2026-10-27) optional function on_print(world, line),
    called for every line the robot code prints, at that simulated moment.
    Keep it short and do not move the robot in it.
    grab_sensor: ADDED (2026-10-27) False = the grab distance sensor
    (config.GRAB_TOF) does not start, as if it were not wired, so
    TaskSensors leaves tasks.grab_tof as None."""
    _setup_paths()
    _forget_robot_modules()
    from sim.fakes import _bus
    from sim.arena import build_arena, sim_name
    from sim.devices import SimToF
    from sim.world import RobotSpec, SimStop
    import config
    import control.task_sensors as task_sensors

    track = build_arena(pad_fill_white=pad_fill_white)
    spec = RobotSpec(**(spec_overrides or {}))
    if pose is None:
        nx, ny = track.nodes[at]
        th = math.radians(HEADINGS[heading] if isinstance(heading, str) else heading)
        # rear axle `back` metres behind the node, facing `heading`
        # (back = 0.20 = LINE_SENSOR_AHEAD: the line sensors are on the node)
        pose = (nx - back * math.cos(th), ny - back * math.sin(th), th)
    world = make_world(track, spec, pose, seconds, stop_on_collision)
    _bus.world = world

    # Task sensors: magnet (real driver on the fake ADC), IR receiver (real
    # driver on the fake pin), front distance sensor (simulated object).
    # CHANGED (2026-10-27): the rear sensor (config.REAR_TOF) looks BACKWARDS
    # from the robot's tail; the grab sensor looks forward from the nose.
    task_sensors.make_rear_tof = lambda: SimToF(world, facing="rear")
    if grab_sensor:
        task_sensors.make_grab_tof = lambda: SimToF(world)
    else:
        # ADDED (2026-10-27): the grab sensor "does not start" (see the docstring)
        task_sensors.make_grab_tof = _grab_sensor_missing
    pad_poles = dict(DEFAULT_POLES)
    for name, pole in (poles or {}).items():
        pad_poles[sim_name(name)] = pole
    # ADDED (2026-10-26): servo2 at (or within 2 degrees of) the low end of
    # its safe range counts as "gripper closed": in front of a pick-up box
    # that takes the box (sim/world.py pwm_output()).
    from actuators.servo import Servo
    grip_low = config.SERVO_SAFE_RANGE["servo2"][0] + 2
    grip_closed_duty = int(Servo.MIN_DUTY + grip_low / Servo.MAX_ANGLE * (Servo.MAX_DUTY - Servo.MIN_DUTY))
    world.setup_task_world(mag_pin=config.MAG_PIN, pad_poles=pad_poles,
                           mag_north_sign=config.MAG_NORTH_SIGN,
                           boxes=[sim_name(b) for b in boxes], ir_pin=config.IR_RX_PIN, light=light,
                           ir_when="red" if config.TRAFFIC_RED_WHEN_IR else "green",
                           grip_pin=config.SERVO_PINS["servo2"], grip_closed_duty=grip_closed_duty)

    # CHANGED (2026-10-23): START_STOP_SENSOR = "push" presses the push button.
    button_pin = {"left": config.CRASH_LEFT_PIN, "right": config.CRASH_RIGHT_PIN,
                  "push": config.PUSH_BUTTON_PIN}[config.START_STOP_SENSOR]
    for t in presses:
        world.press(button_pin, t, hold_ms=(press_hold or {}).get(t, 80))

    module_name, func_name = entry.split(":") if ":" in entry else (entry, "main")
    # main.py's functions.reset_map() rewrites MAP.py in the CURRENT folder:
    # run in a temporary folder with a copy, so the repository is not touched.
    workdir = tempfile.mkdtemp(prefix="idp_sim_")
    if os.path.exists(os.path.join(ROOT, "MAP.py")):
        shutil.copy(os.path.join(ROOT, "MAP.py"), workdir)
    old_cwd, old_stdout = os.getcwd(), sys.stdout
    out = _TimedOutput(world, old_stdout, quiet, on_print)    # CHANGED (2026-10-27): on_print
    os.chdir(workdir)
    sys.stdout = out
    try:
        module = __import__(module_name, fromlist=[func_name])
        getattr(module, func_name)(**(kwargs or {}))
        reason = "robot code returned"
    except SimStop as e:
        reason = e.reason
    except Exception:
        reason = "robot code crashed:\n" + traceback.format_exc()
    finally:
        sys.stdout = old_stdout
        os.chdir(old_cwd)
        shutil.rmtree(workdir, ignore_errors=True)
        _bus.world = None
    if world.stopped is None:
        world.stopped = reason.splitlines()[0]
    if png:
        from sim.render import draw_run
        draw_run(track, world, out.notes, png, title="%s  (%s)" % (entry, world.stopped))
    return world, out.notes, reason


def main(argv=None):
    ap = argparse.ArgumentParser(description="Run the robot code against the simulated table.")
    ap.add_argument("entry", nargs="?", default="main:main", help="module:function to run (default main:main)")
    ap.add_argument("--time", type=float, default=240.0, help="simulated seconds (default 240)")
    ap.add_argument("--press", nargs="*", default=["0.3"],
                    help="times (s) to press the start/stop button; 'none' = never")
    ap.add_argument("--at", default="START", help="start node (default START)")
    ap.add_argument("--heading", default="north", help="north/east/south/west or degrees (0 = east)")
    ap.add_argument("--back", type=float, default=0.20,
                    help="rear axle this far behind the start node (m, default 0.20 = line sensors on the node)")
    ap.add_argument("--pose", help="x,y,heading_deg of the rear axle (overrides --at)")
    ap.add_argument("--spec", nargs="*", help="robot geometry overrides, e.g. sensor_ahead=0.18 vmax=0.18")
    ap.add_argument("--kwargs", help="JSON dict of keyword arguments for the entry function")
    ap.add_argument("--png", default="sim_run.png", help="picture of the run ('' = none)")
    ap.add_argument("--quiet", action="store_true", help="do not echo the robot's prints")
    ap.add_argument("--no-collision-stop", action="store_true", help="keep going after hitting a rack / the edge")
    ap.add_argument("--dark-pads", action="store_true", help="coloured pad fill reads black (default: white)")
    ap.add_argument("--boxes", nargs="*", default=[], help="rack slots that already hold a box, e.g. ORANGE_W1")
    ap.add_argument("--poles", nargs="*", default=[], help="magnet pole per pad, e.g. GREEN=south")
    ap.add_argument("--light", default="8,8", help="traffic light red_s,green_s[,phase_s], or none (default 8,8)")
    a = ap.parse_args(argv)

    presses = [] if (a.press and a.press[0].lower() == "none") else [float(x) for x in a.press]
    heading = a.heading if a.heading in HEADINGS else float(a.heading)
    pose = None
    if a.pose:
        px, py, ph = (float(v) for v in a.pose.split(","))
        pose = (px, py, math.radians(ph))
    world, notes, reason = run(a.entry, a.time, presses, a.at, heading, a.back, pose,
                               _parse_spec(a.spec), json.loads(a.kwargs) if a.kwargs else None,
                               a.png, a.quiet, not a.no_collision_stop, not a.dark_pads, a.boxes,
                               dict(p.split("=", 1) for p in a.poles), _parse_light(a.light))
    print("-" * 60)
    print("end after %.2f s simulated: %s" % (world.t(), reason))
    print("distance driven %.2f m; final rear-axle pose (%.2f, %.2f) heading %.0f deg" % (
        world.distance, world.x, world.y, math.degrees(world.th) % 360))
    if a.png:
        print("picture:", os.path.abspath(a.png))


if __name__ == "__main__":
    main()
