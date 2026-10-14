"""
Step 6: 2 x SEN0138 crash sensors.

Wiring: VCC -> 3V3_OUT (physical 36), GND -> GND (physical 38), DIG/S (signal)
straight to the Pico, no divider:
    Crash Sensor L -> GP8 (physical 11)     Crash Sensor R -> GP9 (physical 12)
(moved from GP6/GP7 in the schematic on 2026-10-14).
Only if the sensors are powered from 5V instead: a divider (10k series, 20k
to GND) is REQUIRED between DIG/S and the Pico pin.
Measure the signal with a multimeter first: about 3.3 V released, 0 V pressed.

Checks: raw 1 / pressed False at rest; press a lever -> raw 0, pressed True,
one hit counted per press; tap quickly -> still exactly one hit.
Stop with Ctrl-C.
"""

from utime import sleep_ms, ticks_ms, ticks_diff

from sensors.crash import BumperPair


def test_crash(seconds=None):
    bumpers = BumperPair.from_config()
    start = ticks_ms()
    last_print = start
    while seconds is None or ticks_diff(ticks_ms(), start) < seconds * 1000:
        now = ticks_ms()
        bumpers.update(now)
        hit = bumpers.consume_hits()
        if hit is not None:
            print(">>> hit:", hit)
        if ticks_diff(now, last_print) >= 200:
            last_print = now
            print("left raw %d pressed %s hits %d   right raw %d pressed %s hits %d   contact %s" % (
                bumpers.left.raw(), bumpers.left.pressed, bumpers.left.hits,
                bumpers.right.raw(), bumpers.right.pressed, bumpers.right.hits,
                bumpers.which()))
        sleep_ms(10)


if __name__ == "__main__":
    test_crash()
