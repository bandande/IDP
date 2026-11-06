from utime import sleep_ms, ticks_ms, ticks_diff

from sensors.magnetic import HallSensor


def test_magnetic(seconds=None):
    mag = HallSensor.from_config()
    if mag.mode == "analog":
        print("zero-field baseline: %d mV (keep magnets away while this is measured)" % mag.calibrate_baseline())
    start = ticks_ms()
    last_print = start
    while seconds is None or ticks_diff(ticks_ms(), start) < seconds * 1000:
        now = ticks_ms()
        mag.update(now)
        if mag.consume_detection():
            print(">>> magnet detected", "" if mag.field_mv is None else "(field %+d mV)" % mag.field_mv)
        if ticks_diff(now, last_print) >= 200:
            last_print = now
            if mag.mode == "analog":
                print("%4d mV   field %+5d mV   %s" % (mag.mv, mag.field_mv,
                                                       "MAGNET" if mag.detected else "-"))
            else:
                print("level %d   %s" % (mag.raw(), "MAGNET" if mag.detected else "-"))
        sleep_ms(10)


if __name__ == "__main__":
    test_magnetic()
