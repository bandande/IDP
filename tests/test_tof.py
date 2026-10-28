"""
Step 3: 2 x VL53L0X time-of-flight sensors.

Wiring per sensor (boards labelled + - C D): + -> 3V3_OUT (physical 36,
never 5V), - -> GND (physical 38), D = SDA and C = SCL.
CHANGED (2026-10-27): the team re-wired the SDA wires; the pins are in
config.TOF_SENSORS, and both buses are now software I2C (SoftI2C):
    Distance Sensor 1 "dist1" = GRAB sensor (front, through the gripper):
        D -> GP2 (physical 4), C -> GP17 (physical 22)
        (D must NOT also touch VCC: the schematic draws them on one wire by mistake)
    Distance Sensor 2 "dist2" = REAR sensor (back of the robot):
        D -> GP0 (physical 1), C -> GP11 (physical 15)
No XSHUT pin on these boards (one sensor per bus instead). No extra parts:
the Pico enables its internal pull-ups; add 4.7k pull-ups to 3V3 if
readings are flaky on long wires (I2C_FREQ_HZ is already 100000).
Peel the protective film off the sensor window.

Checks:
  1. "available True" for both sensors (else see last_error and run test_wiring).
  2. Hand / box at 100-500 mm: distance follows it, status 11.
  3. Nothing in front: distance None, raw 8190/8191 or status 4.
Offset calibration: calibrate_offset("dist1", 100) with a flat target exactly
100 mm in front of the sensor, then copy the printed offset into config.py.
Stop with Ctrl-C.
"""

from utime import sleep_ms, ticks_ms, ticks_diff

from sensors.tof import tof_from_config


def test_tof(seconds=None):
    tofs = tof_from_config()
    while True:
        for name, s in tofs.items():
            # print("%s: address 0x%02X available %s %s" % (
            #     name, s.address, s.available, "" if s.available else s.last_error))
            print(s.processor.value)
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
                parts.append("%s: %s mm (raw %s, status %s, errors %d)" % (
                    name, s.get_distance(now), s.raw_mm, s.range_status, s.error_count))
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
