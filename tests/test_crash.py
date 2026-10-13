"""
Step 4: 2 x SEN0138 crash sensors (the only sensors that need extra parts).

Wiring per sensor: VCC -> 5V rail of the robot board (VBUS, physical pin 40, is
only 5V while USB is plugged in), GND -> GND, OUT -> divider -> Pico pin:
    OUT --[10k]--+-- GPIO (left GP16 = physical 21, right GP17 = physical 22)
                 |
               [20k]
                 |
                GND
Before connecting the GPIO wire, measure the divider midpoint with a
multimeter: about 2.2-3.3 V released, about 0 V pressed. Never above 3.3 V.

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
