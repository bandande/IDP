"""
Step 2: 4 x SEN0017 line sensors.

Wiring (team schematic): VCC -> 3V3_OUT (physical 36), GND -> GND (physical 38),
OUT, left to right across the robot:
    Edge Left   -> GP20 (physical 26)      Centre Right -> GP19 (physical 25)
    Centre Left -> GP21 (physical 27)      Edge Right   -> GP18 (physical 24)
No extra parts.

Checks:
  1. Over black floor the pattern is "....".
  2. Hold white tape under each sensor in turn: the matching character becomes
     '#', left to right. Wrong one flips -> fix the pin order in config.py.
     All inverted -> set LINE_WHITE_IS_HIGH = False.
  3. Centre the robot on a line: ".##." with error +0.00. Slide it sideways:
     line to the left of the robot -> negative error.
  4. Push the robot slowly over a junction: ENTER then EXIT events.
Stop with Ctrl-C.
"""

from utime import sleep_ms, ticks_ms, ticks_diff

from sensors.line_sensor import LineSensorArray
from sensors.line_processing import LineTracker, JunctionDetector, bits_to_str


def test_line_sensors(seconds=None):
    line = LineSensorArray.from_config()
    tracker = LineTracker.from_config()
    junctions = JunctionDetector.from_config()
    print("line sensor pins left to right:", line.pin_numbers)
    start = ticks_ms()
    last_print = start
    while seconds is None or ticks_diff(ticks_ms(), start) < seconds * 1000:
        now = ticks_ms()
        bits = line.read()
        error = tracker.update(bits, now)
        event = junctions.update(bits, now)
        if event is not None:
            print(">>>", event)
        if tracker.just_lost:
            print(">>> line lost")
        if ticks_diff(now, last_print) >= 100:
            last_print = now
            print("pattern %s  raw %s  error %+.2f  %s" % (
                bits_to_str(bits), line.read_raw(), error, tracker.state))
        sleep_ms(10)


if __name__ == "__main__":
    test_line_sensors()
