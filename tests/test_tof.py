from utime import sleep_ms, ticks_ms, ticks_diff

from sensors.tof import tof_from_config


def test_tof(seconds=None):
    tofs = tof_from_config()
    for name, s in tofs.items():
        print("%s: address 0x%02X available %s %s" % (
            name, s.address, s.available, "" if s.available else s.last_error))
    start = ticks_ms()
    last_print = start
    while seconds is None or ticks_diff(ticks_ms(), start) < seconds * 1000:
        now = ticks_ms()
        for s in tofs.values():
            s.poll(now)
        if ticks_diff(now, last_print) >= 100:
            last_print = now
            parts = []
            for name, s in tofs.items():
                parts.append("%s: %s mm (raw %s, status %s, errors %d, restarts %d)" % (
                    name, s.get_distance(now), s.raw_mm, s.range_status, s.error_count, s.restart_count))
            print("   ".join(parts))
        sleep_ms(5)


def calibrate_offset(name, true_mm, samples=50):
    """Average raw readings of a flat target at true_mm and print the offset to use."""
    tofs = tof_from_config()
    s = tofs[name]
    if not s.available:
        print("sensor not available:", s.last_error)
        return None
    total = 0
    count = 0
    start = ticks_ms()
    while count < samples and ticks_diff(ticks_ms(), start) < 10000:
        if s.poll() and s.range_status == 11 and s.raw_mm < 8190:
            total += s.raw_mm
            count += 1
        sleep_ms(2)
    if count == 0:
        print("no valid readings")
        return None
    offset = true_mm - total // count
    print("mean raw %d mm over %d samples -> set offset_mm = %d for '%s'" % (
        total // count, count, offset, name))
    return offset


if __name__ == "__main__":
    test_tof()
