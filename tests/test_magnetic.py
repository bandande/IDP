"""
Step 6: 3-wire Hall (magnetic) sensor.

Wiring: + -> 3V3(OUT) (physical pin 36), - -> GND (e.g. AGND, physical 33),
signal -> GP26 (physical 31, ADC0). No extra parts.

Checks (analog module, config.MAG_MODE = "analog"):
  1. With no magnet nearby the field stays within about +/-100 mV of zero.
  2. A magnet held within 1 cm of the chip face: "MAGNET", field several
     hundred mV; the other pole moves the field the other way.
  3. Take it away: back to "-" within a second.
The live curve in tools/sensor_viewer.py (Magnetic tab) shows the same data.
Stop with Ctrl-C.
"""

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
