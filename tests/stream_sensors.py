"""
Stream every sensor reading to the PC as JSON lines, for tools/sensor_viewer.py.

Lines starting with "@H " describe the setup (sent once), lines starting with
"@S " are samples (every period_ms), "@E " marks the end. Anything else is
ordinary text (errors, tracebacks). The viewer starts this automatically; to
run it by hand in the REPL:

    from tests.stream_sensors import stream
    stream()                                   # wiring from config.py
    stream(line_pins=[16, 18, 19, 20], crash_pins=[], ir_pin=22, use_tof=False)

line_pins: 4 GPIOs left to right, or [] to disable. crash_pins: [left, right]
or []. ir_pin: GPIO or -1. mag_pin: GPIO or -1; on an ADC pin (GP26-28) the
magnetic sensor is always read as an analog voltage, so both digital and analog
modules can be seen. seconds: stop after this long (None = until Ctrl-C).
The pin monitor shows the raw level of every header
GPIO except the motor, servo and LED pins and the I2C pins in use. Unused
pins get the internal pull-up, so an unconnected pin reads a steady 1.
Stop with Ctrl-C.
"""

import json
from machine import Pin
from utime import sleep_ms, ticks_ms, ticks_diff

import config
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


def _emit(tag, obj):
    print(tag + json.dumps(obj))


def stream(line_pins=None, crash_pins=None, ir_pin=None, use_tof=True, monitor=None, period_ms=50,
           seconds=None, mag_pin=None):
    if line_pins is None:
        line_pins = [config.LINE_OUTER_LEFT_PIN, config.LINE_INNER_LEFT_PIN,
                     config.LINE_INNER_RIGHT_PIN, config.LINE_OUTER_RIGHT_PIN]
    if crash_pins is None:
        crash_pins = [config.CRASH_LEFT_PIN, config.CRASH_RIGHT_PIN]
    if ir_pin is None:
        ir_pin = config.IR_RX_PIN
    if monitor is None:
        monitor = default_monitor_pins(use_tof)
    errors = []

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

    if mag_pin is None:
        mag_pin = config.MAG_PIN
    mag = None
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
        "errors": errors,
    })

    events = []
    start = last_emit = ticks_ms()
    try:
        while seconds is None or ticks_diff(ticks_ms(), start) < seconds * 1000:
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
                if events:
                    msg["ev"] = events
                    events = []
                _emit("@S ", msg)
            sleep_ms(5)
    except KeyboardInterrupt:
        pass   # Ctrl-C from the viewer or the REPL: stop quietly
    finally:
        if ir is not None:
            ir.deinit()
        if bumpers is not None:
            bumpers.left.deinit()
            bumpers.right.deinit()
        for s in tofs.values():
            s.stop()
        print("@E stopped")


if __name__ == "__main__":
    stream()
