import json
import sys
from machine import Pin
from utime import sleep_ms, ticks_ms, ticks_diff

try:
    import select
except ImportError:
    import uselect as select

import config
from actuators.servo import Servo
from sensors.line_sensor import LineSensorArray
from sensors.line_processing import LineTracker, JunctionDetector
from sensors.tof import tof_from_config, make_i2c
from sensors.crash import CrashSensor, BumperPair
from sensors.ir_receiver import IRReceiver, decode_nec, frame_signature
from sensors.magnetic import HallSensor, ADC_PINS


def default_monitor_pins(use_tof=True):
    # Never touch motor, servo or LED pins: a pulled-up DIR/PWM input could start a motor.
    busy = set(config.BOARD_PINS) | set(config.LED_PINS.values())
    if use_tof:
        for t in config.TOF_SENSORS:
            busy.add(t["sda"])
            busy.add(t["scl"])
            if t["xshut"] is not None:
                busy.add(t["xshut"])
    return [gp for gp in range(29) if config.physical_pin(gp) is not None and gp not in busy]


def _output_pin_use(gp):
    """What the robot uses GPIO `gp` for if it is one of
    its OUTPUT pins - config.BOARD_PINS (motors, servos) or config.LED_PINS -
    else None. A sensor must never be set up on such a pin: the sensor
    classes and the pin monitor make the pin an input with the internal
    pull-up, which can start a motor, move a servo or light an LED."""
    if gp in config.BOARD_PINS:
        return config.BOARD_PINS[gp]
    for colour, pin in config.LED_PINS.items():
        if pin == gp:
            return colour + " LED"
    return None


def _refuse_output_pins(what, pins, errors):
    """True if any GPIO in `pins` is a robot output pin
    (see _output_pin_use). Each such pin adds one message to `errors` (the
    list the viewer shows in its log). `what` names the sensor."""
    refused = False
    for gp in pins:
        use = _output_pin_use(gp)
        if use is not None:
            errors.append("%s on GP%d refused: that pin is the robot's %s (an output, not a sensor input)"
                          % (what, gp, use))
            refused = True
    return refused


def _emit(tag, obj):
    print(tag + json.dumps(obj))


class CommandReader:
    """Collects the command lines the PC types while the
    stream runs, without ever waiting for them.

    lines() returns the complete lines received since the last call (usually
    none). Ctrl-C is not a character here: it still stops the stream."""

    def __init__(self):
        self.text = ""
        try:
            self.poller = select.poll()
            self.poller.register(sys.stdin, select.POLLIN)
        except Exception:
            self.poller = None      # no stdin polling on this port: commands are ignored

    def lines(self):
        done = []
        if self.poller is None:
            return done
        count = 0
        # poll(0) = "is a character waiting?" without waiting. At most 64
        # characters per call, so the sensor loop is never held up for long.
        while count < 64 and self.poller.poll(0):
            ch = sys.stdin.read(1)
            count += 1
            if ch == "\n" or ch == "\r":
                if self.text:
                    done.append(self.text)
                    self.text = ""
            else:
                self.text += ch
        return done


def servo_command(line, servos, events, free=False):
    """Carry out one command line, e.g. "servo1 135" or
    "servo1 off". Problems are reported as events (shown in the viewer's log).
    free = calibration mode (see stream(servo_free=True))."""
    parts = line.split()
    if len(parts) != 2 or parts[0] not in servos:
        events.append("unknown command %r (expected e.g. 'servo1 135' or 'servo1 off')" % line)
        return
    servo = servos[parts[0]]
    if parts[1] == "off":
        servo.off()
        events.append("%s off (limp)" % parts[0])
        return
    if parts[1] == "reset":
        # (config.SERVO_RESET, or straight to the middle of the safe range).
        # The stream pauses while it runs.
        angle = servo.reset(config.SERVO_RESET_SETTLE_MS, config.SERVO_RESET_STEP_MS)
        if servo.reset_path is None:
            events.append("%s reset: straight to the middle of its safe range, %d deg" % (parts[0], angle))
        else:
            events.append("%s reset: %d -> %d deg, slowly" % (parts[0], servo.reset_path[0], angle))
        return
    try:
        angle = float(parts[1])
    except ValueError:
        events.append("bad angle in %r" % line)
        return
    if servo.angle() is None:
        if free:
            events.append("%s started at %d deg (calibration mode)" % (parts[0], servo.start_at(angle)))
            return
        events.append("%s has no pulses yet: send '%s reset' first" % (parts[0], parts[0]))
        return
    if angle < servo.low or angle > servo.high:
        events.append("%s: %s deg is outside the safe range %d-%d, clamped" % (
            parts[0], parts[1], servo.low, servo.high))
    servo.set_angle(angle)     # clamped into the safe range by the Servo class


# ---------------------------------------------------------------------------
DRIVE_MAX_MS = 2000     # one "drive" / "back" command never runs longer than this


def _signed(ms):
    """600 -> "+600", -200 -> "-200" (for the event lines)."""
    if ms >= 0:
        return "+%d" % ms
    return "%d" % ms


class DriveSteps:
    """Short timed drives for the drop-off calibration.

    start(sign, ms) switches both wheels on at `speed` % (forwards for
    sign +1, backwards for sign -1). update(), called in every loop of the
    stream, switches them off again once `ms` have passed. Nothing here
    waits, so the stream keeps sending samples while the robot moves.
    stop() switches the wheels off at once.

    net_ms() = net forward drive time since the stream started: the time
    each move REALLY ran (from wheels on to wheels off), forwards +,
    backwards -. A move cut short by stop() or replaced by a new start()
    counts only the time it ran.

    The motors are created on the first start(), so a stream that never
    gets a drive command does not touch the motor pins at all."""

    def __init__(self, speed):
        self.speed = max(1, min(100, speed))   # 1 - 100 %, like control/dropoff.py
        self.left = None        # the two actuators.motor.Motor objects (made by _make_motors)
        self.right = None
        self.sign = 0           # +1 = driving forwards, -1 = backwards, 0 = standing still
        self.started = 0        # ticks_ms() when the present move started
        self.length = 0         # how long the present move should last (ms)
        self.done_ms = 0        # net forward ms of all moves that have ended

    def _make_motors(self):
        """Create the motor objects the first time they are needed.
        actuators/motor.py Motor with invert=False, 1000 Hz and min_duty=0
        drives the pins EXACTLY like the team's motor.py Motor that main.py
        and control/dropoff.py use: DIR pin 0 = forwards, 1 = backwards,
        PWM duty = speed % of 65535. (motor.py itself cannot be imported
        here: it imports the whole of functions.py.) The pins are the ones
        main.py uses: left wheel DIR GP4 / PWM GP5, right wheel DIR GP7 /
        PWM GP6 (config.MOTOR_* has the same numbers). Creating a Motor sets
        DIR = 0 and duty 0, so nothing moves yet."""
        from actuators.motor import Motor
        if self.left is None:
            self.left = Motor(config.MOTOR_LEFT_DIR_PIN, config.MOTOR_LEFT_PWM_PIN,
                              invert=False, freq=1000, min_duty=0)
        if self.right is None:
            self.right = Motor(config.MOTOR_RIGHT_DIR_PIN, config.MOTOR_RIGHT_PWM_PIN,
                               invert=False, freq=1000, min_duty=0)

    def moving(self):
        return self.sign != 0

    def net_ms(self):
        """Net forward ms, including the part of the present move run so far."""
        if self.sign == 0:
            return self.done_ms
        return self.done_ms + self.sign * ticks_diff(ticks_ms(), self.started)

    def _count(self, now):
        """Add the time the present move has run (until `now`) to done_ms and
        mark the robot as standing. Returns that time. The wheels are not
        touched here."""
        ran = ticks_diff(now, self.started)
        self.done_ms += self.sign * ran
        self.sign = 0
        return ran

    def start(self, sign, ms):
        """Wheels on, forwards (sign +1) or backwards (sign -1), for `ms` ms.
        A move that is still running is counted up to now and replaced
        (the wheels are not switched off in between). Returns how long the
        replaced move ran, or None if the robot was standing."""
        self._make_motors()               # may raise: then nothing moves
        now = ticks_ms()
        replaced = None
        if self.sign != 0:
            replaced = self._count(now)
        self.left.set(sign * self.speed)
        self.right.set(sign * self.speed)
        self.sign = sign
        self.started = now
        self.length = ms
        return replaced

    def update(self):
        """Call in every loop: wheels off once the present move's time is up."""
        if self.sign == 0:
            return
        now = ticks_ms()
        if ticks_diff(now, self.started) >= self.length:
            self._wheels_off()
            self._count(now)

    def stop(self):
        """Wheels off at once. Returns how long the move ran, or None if the
        robot was standing anyway."""
        if self.sign == 0:
            return None
        self._wheels_off()
        return self._count(ticks_ms())

    def zero(self):
        """Stop, then start counting from 0 again (a new attempt)."""
        self.stop()
        self.done_ms = 0

    def _wheels_off(self):
        """PWM duty 0 on every motor that exists. Each one separately, so a
        failure on one cannot leave the other running."""
        for motor in (self.left, self.right):
            if motor is not None:
                try:
                    motor.stop()
                except Exception:
                    pass

    def off(self):
        """For the stream's `finally`: wheels off, whatever happened."""
        self._wheels_off()
        self.sign = 0


def drive_command(line, drive, events):
    """Carry out a drive command of the drop-off
    calibration: "drive <ms>", "back <ms>", "stop" or "zero". Returns True if
    `line` was one of them (otherwise the caller tries the other commands).
    Every command adds one event line (shown in the viewer's log)."""
    parts = line.split()
    if not parts or parts[0] not in ("drive", "back", "stop", "zero"):
        return False
    word = parts[0]
    if word == "stop":
        ran = drive.stop()
        if ran is None:
            events.append("stop: not driving (net %s ms)" % _signed(drive.net_ms()))
        else:
            events.append("stop after %d ms (net %s ms)" % (ran, _signed(drive.net_ms())))
        return True
    if word == "zero":
        ran = drive.stop()
        drive.zero()
        if ran is None:
            events.append("net forward count reset to 0 ms")
        else:
            events.append("stop after %d ms, net forward count reset to 0 ms" % ran)
        return True
    # "drive <ms>" or "back <ms>"
    if len(parts) != 2:
        events.append("%s needs a time in ms, e.g. '%s 100'" % (word, word))
        return True
    try:
        ms = int(float(parts[1]))
    except ValueError:
        events.append("bad time in %r" % line)
        return True
    if ms <= 0:
        events.append("%s: the time must be more than 0 ms" % word)
        return True
    note = ""
    if ms > DRIVE_MAX_MS:
        ms = DRIVE_MAX_MS
        note = ", capped at %d ms" % DRIVE_MAX_MS
    sign = 1 if word == "drive" else -1
    try:
        replaced = drive.start(sign, ms)
    except Exception as e:
        drive.off()
        events.append("%s: the motors could not be started: %r" % (word, e))
        return True
    if replaced is not None:
        note += ", replaces the move before after %d ms" % replaced
    events.append("drive %s %d ms (net %s ms when done%s)" % (
        "forward" if sign > 0 else "back", ms, _signed(drive.done_ms + sign * ms), note))
    return True


def release_command(servos, events):
    """"release" - let go of the box, the same move as
    the robot's drop-off (actuators/gripper.py release_box): servo2 opens by
    config.GRIPPER_RELEASE_DEG from its present angle, one degree at a time
    (Servo.move_slowly). It never goes past the open (high) end of servo2's
    safe range, and never moves the other way (closing)."""
    servo = servos.get("servo2")
    if servo is None:
        events.append("release: there is no servo2 in this stream")
        return
    start = servo.angle()
    if start is None:
        events.append("release: servo2 is limp (no pulses) - nothing to let go")
        return
    top = servo.high
    safe = config.SERVO_SAFE_RANGE.get("servo2")
    if safe is not None and safe[1] < top:
        top = safe[1]       # calibration mode (servo_free): still stop at config's open end
    target = min(start + config.GRIPPER_RELEASE_DEG, top)
    if target <= start:
        events.append("release: servo2 is already at the open end of its safe range (%d deg)" % start)
        return
    servo.move_slowly(target, config.GRIPPER_STEP_MS)
    events.append("release: servo2 %d -> %d deg" % (start, servo.angle()))


def grab_command(servos, events, free=False):
    """"grab" - pick a box up from the ground and lift
    it, e.g. one that slipped out during the drop-off calibration:
        1. servo2 to its high end (gripper open, before the arm goes down)
        2. servo1 to its low end  (arm down)
        3. servo2 to its low end  (gripper closes on the box)
        4. servo1 to its high end (arm up, box lifted)
    Steps 2 - 4 are the same moves, in the same order, as
    actuators/gripper.py grab_box. Every move goes one degree at a time
    (Servo.move_slowly) inside the safe range, with config.GRIPPER_PAUSE_MS
    after it. One event per step and one at the end. WAITS about 2 s.
    Nothing moves if a servo has no pulses, or in calibration mode
    (free=True: the safe ranges are not used there, the ends would be
    0 / 270)."""
    s1 = servos.get("servo1")
    s2 = servos.get("servo2")
    if s1 is None or s2 is None:
        events.append("grab refused: this stream has no servo1 / servo2")
        return
    if free:
        events.append("grab refused: not available in calibration mode (no safe ranges)")
        return
    for name, servo in (("servo1", s1), ("servo2", s2)):
        if servo.angle() is None:
            events.append("grab refused: %s has no pulses - send '%s reset' first" % (name, name))
            return
    # Step 1 only: if servo2 is BELOW its safe low end (e.g. still at the
    # push angle), let this move start from there, so it opens one degree
    # at a time instead of first jumping up to the low end.
    low2 = min(s2.low, int(round(s2.angle())))
    steps = (
        (s2, "servo2", s2.high, low2, "gripper open"),
        (s1, "servo1", s1.low, None, "arm down"),
        (s2, "servo2", s2.low, None, "gripper closed"),
        (s1, "servo1", s1.high, None, "arm up"),
    )
    number = 0
    for servo, name, angle, low, what in steps:
        number += 1
        servo.move_slowly(angle, config.GRIPPER_STEP_MS, low)
        sleep_ms(config.GRIPPER_PAUSE_MS)
        events.append("grab %d/4: %s (%s %d deg)" % (number, what, name, servo.angle()))
    events.append("grab done: servo1 %d deg, servo2 %d deg (box lifted)" % (s1.angle(), s2.angle()))


def handle_command(line, servos, drive, events, free=False):
    """One command line typed by the PC: a drive command
    (drive_command), "release", "grab" or a servo command (servo_command)."""
    if drive_command(line, drive, events):
        return
    parts = line.split()
    # "release", "grab" and "servoN reset" WAIT while the servos move
    # (0.1 - 2.5 s). Stop a running drive first, so the wheels never run
    # longer than asked; the time driven so far counts as usual.
    waits = parts == ["release"] or parts == ["grab"] or (len(parts) == 2 and parts[1] == "reset")
    if waits and drive.moving():
        ran = drive.stop()
        events.append("stop after %d ms before %r (net %s ms)" % (ran, line, _signed(drive.net_ms())))
    if parts == ["release"]:
        release_command(servos, events)
    elif parts == ["grab"]:
        grab_command(servos, events, free)
    else:
        servo_command(line, servos, events, free)


def stream(line_pins=None, crash_pins=None, ir_pin=None, use_tof=True, monitor=None, period_ms=50,
           seconds=None, mag_pin=None, servo_pins=None, servo_free=False):
    if line_pins is None:
        line_pins = [config.LINE_OUTER_LEFT_PIN, config.LINE_INNER_LEFT_PIN,
                     config.LINE_INNER_RIGHT_PIN, config.LINE_OUTER_RIGHT_PIN]
    if crash_pins is None:
        crash_pins = [config.CRASH_LEFT_PIN, config.CRASH_RIGHT_PIN]
    if ir_pin is None:
        ir_pin = config.IR_RX_PIN
    if mag_pin is None:
        mag_pin = config.MAG_PIN
    if monitor is None:
        monitor = default_monitor_pins(use_tof)
    errors = []

    if _refuse_output_pins("line sensor", line_pins, errors):
        line_pins = []
    if _refuse_output_pins("crash sensor", crash_pins, errors):
        crash_pins = []
    if ir_pin is not None and ir_pin >= 0 and _refuse_output_pins("IR receiver", [ir_pin], errors):
        ir_pin = -1
    if mag_pin is not None and mag_pin >= 0 and _refuse_output_pins("magnetic sensor", [mag_pin], errors):
        mag_pin = -1
    monitor = [gp for gp in monitor if not _refuse_output_pins("pin monitor", [gp], errors)]

    line = tracker = junctions = None
    if len(line_pins) == 4:
        try:
            line = LineSensorArray(line_pins, config.LINE_WHITE_IS_HIGH, config.LINE_SAMPLES_PER_READ)
            tracker = LineTracker.from_config()
            junctions = JunctionDetector.from_config()
        except Exception as e:
            errors.append("line: %r" % (e,))
            line = None
    elif line_pins:
        errors.append("line needs exactly 4 pins, got %d" % len(line_pins))

    bumpers = None
    if len(crash_pins) == 2:
        try:
            bumpers = BumperPair(
                CrashSensor(crash_pins[0], config.CRASH_ACTIVE_LOW, config.CRASH_DEBOUNCE_MS, name="left"),
                CrashSensor(crash_pins[1], config.CRASH_ACTIVE_LOW, config.CRASH_DEBOUNCE_MS, name="right"))
        except Exception as e:
            errors.append("crash: %r" % (e,))
    elif crash_pins:
        errors.append("crash needs exactly 2 pins, got %d" % len(crash_pins))

    mag = None                     # (mag_pin got its default and its pin check above)
    if mag_pin is not None and mag_pin >= 0:
        try:
            if mag_pin in ADC_PINS:
                mag = HallSensor(mag_pin, mode="analog", on_mv=config.MAG_ON_MV, off_mv=config.MAG_OFF_MV)
                mag.calibrate_baseline(30)
            else:
                mag = HallSensor(mag_pin, mode="digital", active_low=config.MAG_ACTIVE_LOW,
                                 debounce_ms=config.MAG_DEBOUNCE_MS)
        except Exception as e:
            errors.append("magnetic: %r" % (e,))
            mag = None

    ir = None
    if ir_pin is not None and ir_pin >= 0:
        try:
            ir = IRReceiver(ir_pin, config.IR_FRAME_GAP_US, present_window_ms=config.IR_PRESENT_WINDOW_MS)
        except Exception as e:
            errors.append("ir: %r" % (e,))

    tofs = {}
    scan = []
    if use_tof:
        # Scan each ToF bus first so the viewer can show what is physically there.
        for t in config.TOF_SENSORS:
            try:
                found = make_i2c(t["i2c_id"], t["sda"], t["scl"], config.I2C_FREQ_HZ).scan()
            except Exception as e:
                found = None
                errors.append("tof %s: I2C scan failed: %r" % (t["name"], e))
            scan.append([t["name"], t["i2c_id"], t["sda"], t["scl"], t["address"], found])
        try:
            tofs = tof_from_config()
        except Exception as e:
            errors.append("tof: %r" % (e,))
        for name, s in tofs.items():
            if not s.available:
                errors.append("tof %s not available: %s" % (name, s.last_error))

    if servo_pins is None:
        servo_pins = config.SERVO_PINS
    servos = {}
    for name in sorted(servo_pins):
        safe = config.SERVO_SAFE_RANGE.get(name)
        if servo_free:
            safe = (0, Servo.MAX_ANGLE)
        elif safe is None:
            errors.append("servo %s left out: no safe range in config.SERVO_SAFE_RANGE" % name)
            continue
        try:
            servos[name] = Servo(servo_pins[name], safe, config.SERVO_RESET.get(name))
            if servo_free:
                servos[name].reset_path = None     # the old reset path belongs to the old hardware
            servos[name].off()
        except Exception as e:
            errors.append("servo %s: %r" % (name, e))
    commands = CommandReader()
    drive = DriveSteps(config.DROP_MOVE_SPEED)

    # One reader per monitored pin; reuse the driver's view of pins it owns.
    readers = []
    for gp in monitor:
        reader = None
        if line is not None and gp in line_pins:
            i = line_pins.index(gp)
            reader = (lambda i=i: line.read_raw()[i])
        elif bumpers is not None and gp == crash_pins[0]:
            reader = bumpers.left.raw
        elif bumpers is not None and gp == crash_pins[1]:
            reader = bumpers.right.raw
        elif mag is not None and gp == mag_pin:
            # Do not re-initialise the pin (that would break the ADC); show the last reading as a level.
            reader = (lambda: 1 if (mag.mv if mag.mode == "analog" else mag.raw() * 3300) > 1650 else 0)
        elif ir is not None and gp == ir_pin:
            reader = ir.raw
        else:
            reader = Pin(gp, Pin.IN, Pin.PULL_UP).value
        readers.append(reader)

    _emit("@H ", {
        "monitor": monitor,
        "phys": [config.physical_pin(gp) for gp in monitor],
        "line": list(line_pins) if line is not None else [],
        "crash": list(crash_pins) if bumpers is not None else [],
        "ir": ir_pin if ir is not None else -1,
        "mag": [mag_pin, mag.mode, mag.baseline_mv, config.MAG_ON_MV, config.MAG_OFF_MV] if mag is not None else None,
        "tof": list(tofs),
        "scan": scan,
        "max_mm": config.TOF_MAX_VALID_MM,
        "servo": [[name, servo_pins[name], servos[name].low, servos[name].high, servos[name].reset_path]
                  for name in sorted(servos)],
        "servo_free": servo_free,       # calibration mode on?
        "drive": [drive.speed, DRIVE_MAX_MS],
        "errors": errors,
    })

    events = []
    start = last_emit = ticks_ms()
    try:
        while seconds is None or ticks_diff(ticks_ms(), start) < seconds * 1000:
            drive.update()      # wheels off when a timed drive is over
            now = ticks_ms()
            bits = 0
            if line is not None:
                bits = line.read()
                tracker.update(bits, now)
                event = junctions.update(bits, now)
                if event is not None:
                    events.append("junction %s left=%s right=%s straight=%s shape=%s" % (
                        event.kind, event.left, event.right, event.straight, event.shape))
                if tracker.just_lost:
                    events.append("line lost")
            for s in tofs.values():
                s.poll(now)
            if bumpers is not None:
                bumpers.update(now)
                hit = bumpers.consume_hits()
                if hit is not None:
                    events.append("crash hit " + hit)
            if mag is not None:
                mag.update(now)
                if mag.consume_detection():
                    events.append("magnet detected (field %s mV, polarity %s)" % (mag.field_mv, mag.polarity))
            if ir is not None:
                ir.process()
                frame = ir.get_frame()
                while frame is not None:
                    n, mark, space, total = frame_signature(frame)
                    events.append("ir frame %d marks, mean mark %d us, mean space %d us, nec=%s, frame=%s" % (
                        n, mark, space, decode_nec(frame), frame[:100]))
                    frame = ir.get_frame()
            for text in commands.lines():
                handle_command(text, servos, drive, events, servo_free)

            if ticks_diff(now, last_emit) >= period_ms:
                last_emit = now
                msg = {"t": now, "p": [r() for r in readers]}
                if line is not None:
                    msg["line"] = [bits, tracker.error, tracker.state,
                                   junctions.in_junction, junctions.exit_count]
                if tofs:
                    msg["tof"] = [[name, s.available, s.get_distance(now), s.raw_mm, s.range_status,
                                   s.error_count, s.processor.sample_count, s.processor.valid_count]
                                  for name, s in tofs.items()]
                if bumpers is not None:
                    msg["crash"] = [[bumpers.left.raw(), bumpers.left.pressed, bumpers.left.hits],
                                    [bumpers.right.raw(), bumpers.right.pressed, bumpers.right.hits]]
                if ir is not None:
                    msg["ir"] = [ir.signal_present(), ir.raw(), ir.edge_count]
                if mag is not None:
                    msg["mag"] = [mag.mv if mag.mode == "analog" else mag.raw(), mag.detected,
                                  mag.field_mv, mag.polarity, mag.detections, mag.baseline_mv]
                if servos:
                    msg["servo"] = [[name, servos[name].angle()] for name in sorted(servos)]
                msg["drive"] = [drive.net_ms(), drive.moving()]
                if events:
                    msg["ev"] = events
                    events = []
                _emit("@S ", msg)
            drive.update()      # checked again, so a drive ends close to its time
            sleep_ms(5)
    except KeyboardInterrupt:
        pass   # Ctrl-C from the viewer or the REPL: stop quietly
    finally:
        drive.off()
        if ir is not None:
            ir.deinit()
        if bumpers is not None:
            bumpers.left.deinit()
            bumpers.right.deinit()
        for s in tofs.values():
            s.stop()
        for s in servos.values():   # servos limp when the stream stops
            s.off()
        print("@E stopped")


if __name__ == "__main__":
    stream()
