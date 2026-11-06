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
