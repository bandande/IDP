from machine import Pin, ADC
from utime import sleep_ms, ticks_ms, ticks_diff

import config
from sensors.tof import make_i2c


def test_wiring(seconds=None):
    problems = config.validate()
    print("config.validate():", problems if problems else "OK")
    for w in config.warnings():
        print("check on the board:", w)

    for spec in config.TOF_SENSORS:
        bus = make_i2c(spec["i2c_id"], spec["sda"], spec["scl"], config.I2C_FREQ_HZ)
        try:
            found = bus.scan()
        except OSError as e:
            found = "scan failed: %r" % (e,)
        print("ToF %-6s SDA GP%d (pin %d) SCL GP%d (pin %d) -> %s   expect [%d]" % (
            spec["name"], spec["sda"], config.physical_pin(spec["sda"]),
            spec["scl"], config.physical_pin(spec["scl"]), found, spec["address"]))

    names = (("line_OL", config.LINE_OUTER_LEFT_PIN), ("line_IL", config.LINE_INNER_LEFT_PIN),
             ("line_IR", config.LINE_INNER_RIGHT_PIN), ("line_OR", config.LINE_OUTER_RIGHT_PIN),
             ("crash_L", config.CRASH_LEFT_PIN), ("crash_R", config.CRASH_RIGHT_PIN),
             ("ir_rx", config.IR_RX_PIN))
    pins = [(name, gp, Pin(gp, Pin.IN)) for name, gp in names]
    print("pin legend: " + "  ".join("%s=GP%d/pin%d" % (n, gp, config.physical_pin(gp))
                                     for n, gp, _ in pins))
    mag_adc = ADC(Pin(config.MAG_PIN)) if config.MAG_PIN in (26, 27, 28) else None
    start = ticks_ms()
    while seconds is None or ticks_diff(ticks_ms(), start) < seconds * 1000:
        mag = "magnet=%d mV" % (mag_adc.read_u16() * 3300 // 65535) if mag_adc else ""
        print("line %d%d%d%d   crash L=%d R=%d   ir=%d   " % tuple(p.value() for _, _, p in pins) + mag)
        sleep_ms(1000)


if __name__ == "__main__":
    test_wiring()
