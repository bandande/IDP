"""
Step 7: every sensor at once, one status line every 200 ms plus events.

Run after each sensor has passed its own test. Useful to check that nothing
interferes (e.g. motor noise on the IR receiver, I2C errors while driving).
Stop with Ctrl-C.
"""

from utime import sleep_ms, ticks_ms, ticks_diff

import config
from sensors.line_sensor import LineSensorArray
from sensors.line_processing import LineTracker, JunctionDetector, bits_to_str
from sensors.tof import tof_from_config
from sensors.crash import BumperPair
from sensors.ir_receiver import IRReceiver, decode_nec
from sensors.magnetic import HallSensor


def test_all_sensors(seconds=None):
    problems = config.validate()
    if problems:
        print("config problems:", problems)
    line = LineSensorArray.from_config()
    tracker = LineTracker.from_config()
    junctions = JunctionDetector.from_config()
    tofs = tof_from_config()
    bumpers = BumperPair.from_config()
    ir = IRReceiver.from_config()
    mag = HallSensor.from_config()
    mag.calibrate_baseline()
    for name, s in tofs.items():
        if not s.available:
            print("ToF %s not available: %s" % (name, s.last_error))

    start = ticks_ms()
    last_print = start
    while seconds is None or ticks_diff(ticks_ms(), start) < seconds * 1000:
        now = ticks_ms()
        bits = line.read()
        error = tracker.update(bits, now)
        event = junctions.update(bits, now)
        for s in tofs.values():
            s.poll(now)
        bumpers.update(now)
        hit = bumpers.consume_hits()
        ir.process()
        frame = ir.get_frame()
        mag.update(now)
        if mag.consume_detection():
            print(">>> magnet detected")

        if event is not None:
            print(">>>", event)
        if hit is not None:
            print(">>> hit:", hit)
        if frame is not None:
            print(">>> IR frame (%d durations): %s" % (len(frame), decode_nec(frame)))
        if ticks_diff(now, last_print) >= 200:
            last_print = now
            dist = "  ".join("%s=%s" % (n, s.get_distance(now)) for n, s in tofs.items())
            print("line %s %+.2f %-9s | %s | bump %s | ir %s | mag %s" % (
                bits_to_str(bits), error, tracker.state, dist, bumpers.which(),
                "on" if ir.signal_present() else "off", "YES" if mag.detected else "-"))
        sleep_ms(5)
    ir.deinit()


if __name__ == "__main__":
    test_all_sensors()
