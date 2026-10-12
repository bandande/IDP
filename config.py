"""
Sensor configuration: every sensor pin number and tuning constant lives here.

Pin numbers are Raspberry Pi Pico GPIO numbers ("GPn"), NOT physical header
pin numbers. Confirm every value with the electrical team before powering up,
and run `validate()` once to catch pin clashes.
"""

# ---------------------------------------------------------------------------
# Line tracking sensors: 4 x DFRobot SEN0017 (digital, white -> 1, black -> 0).
# Power them from 3V3 so their output is safe for the Pico.
# Listed left to right across the robot.
# ---------------------------------------------------------------------------
LINE_OUTER_LEFT_PIN = 12
LINE_INNER_LEFT_PIN = 13
LINE_INNER_RIGHT_PIN = 14
LINE_OUTER_RIGHT_PIN = 15
LINE_WHITE_IS_HIGH = True
LINE_SAMPLES_PER_READ = 3      # majority vote over this many pin reads (use an odd number)
JUNCTION_CONFIRM_SAMPLES = 2   # an outer sensor must see white this many updates in a row
JUNCTION_CLEAR_SAMPLES = 3     # both outer sensors black this many updates to end a junction
LINE_LOST_MS = 150             # all four sensors black for this long -> line lost / end of line

# ---------------------------------------------------------------------------
# Time-of-flight distance sensors: 2 x VL53L0X (I2C). Power from 3V3, not 5V.
# Default: one sensor per hardware I2C bus, so both keep address 0x29.
# Hardware I2C pin options: I2C0 SDA 0/4/8/12/16/20, SCL 1/5/9/13/17/21
#                           I2C1 SDA 2/6/10/14/18/26, SCL 3/7/11/15/19/27
# Set "i2c_id" to None to use SoftI2C on any two free pins.
# Alternative wiring (both sensors on ONE bus): give every sensor an "xshut" pin
# and a distinct "address" (e.g. 0x29 and 0x30).
# ---------------------------------------------------------------------------
I2C_FREQ_HZ = 400000
TOF_SENSORS = (
    {"name": "front", "i2c_id": 0, "sda": 8, "scl": 9, "xshut": None, "address": 0x29, "offset_mm": 0},
    {"name": "side", "i2c_id": 1, "sda": 10, "scl": 11, "xshut": None, "address": 0x29, "offset_mm": 0},
)
TOF_MAX_VALID_MM = 1200        # readings beyond this are reported as "no target" (None)
TOF_MEDIAN_SIZE = 3            # median filter window (odd)
TOF_LONG_RANGE = False         # True -> VCSEL periods 18/14 as in the department example
TOF_STALE_MS = 200             # a reading older than this is considered stale

# ---------------------------------------------------------------------------
# Crash / bump sensors: 2 x DFRobot SEN0138 (micro-switch board, 5V supply ONLY).
# Output: HIGH (5V) when released, LOW when pressed.
# !!! Put a voltage divider (e.g. 10k series, 20k to GND) between the sensor
# !!! output and the Pico pin. RP2040 GPIOs are NOT 5V tolerant.
# ---------------------------------------------------------------------------
CRASH_LEFT_PIN = 16
CRASH_RIGHT_PIN = 17
CRASH_ACTIVE_LOW = True        # pressed -> 0V
CRASH_DEBOUNCE_MS = 15

# ---------------------------------------------------------------------------
# IR receiver: 1 x Vishay TSOP4838 (38 kHz demodulator; output LOW while an IR
# burst is being received). Pins: 1 = OUT, 2 = GND, 3 = VS. Power VS from 3V3
# so the output (internal 30k pull-up to VS) stays at 3.3V.
# ---------------------------------------------------------------------------
IR_RX_PIN = 22
IR_FRAME_GAP_US = 8000         # silence longer than this ends a frame
IR_PRESENT_WINDOW_MS = 150     # signal_present(): any IR activity within this window


# ---------------------------------------------------------------------------
# Sanity check
# ---------------------------------------------------------------------------
_I2C_PINS = {
    0: ((0, 4, 8, 12, 16, 20), (1, 5, 9, 13, 17, 21)),
    1: ((2, 6, 10, 14, 18, 26), (3, 7, 11, 15, 19, 27)),
}
_NOT_ON_HEADER = (23, 24, 25)  # used on the board itself (power-save, VBUS sense, LED / wireless)


def validate():
    """Return a list of human-readable configuration problems (empty if OK)."""
    problems = []
    used = {}

    def claim(pin, what):
        if pin is None:
            return
        if pin < 0 or pin > 28 or pin in _NOT_ON_HEADER:
            problems.append("GP%d (%s) is not a usable header GPIO" % (pin, what))
        if pin in used:
            problems.append("GP%d is used by both %s and %s" % (pin, used[pin], what))
        else:
            used[pin] = what

    claim(LINE_OUTER_LEFT_PIN, "line outer-left")
    claim(LINE_INNER_LEFT_PIN, "line inner-left")
    claim(LINE_INNER_RIGHT_PIN, "line inner-right")
    claim(LINE_OUTER_RIGHT_PIN, "line outer-right")
    claim(CRASH_LEFT_PIN, "crash left")
    claim(CRASH_RIGHT_PIN, "crash right")
    claim(IR_RX_PIN, "IR receiver")

    # Several ToF sensors may share one I2C bus, so claim each bus pin once.
    buses = {}
    for tof in TOF_SENSORS:
        key = (tof["i2c_id"], tof["sda"], tof["scl"])
        buses.setdefault(key, []).append(tof)
        claim(tof["xshut"], "ToF " + tof["name"] + " XSHUT")
    ids_seen = {}
    for (bus_id, sda, scl), members in buses.items():
        label = "/".join(t["name"] for t in members)
        claim(sda, "I2C SDA (" + label + ")")
        claim(scl, "I2C SCL (" + label + ")")
        if bus_id is not None:
            if bus_id not in _I2C_PINS:
                problems.append("I2C id %r does not exist (use 0, 1 or None)" % (bus_id,))
            else:
                sda_ok, scl_ok = _I2C_PINS[bus_id]
                if sda not in sda_ok or scl not in scl_ok:
                    problems.append("GP%d/GP%d cannot be SDA/SCL of hardware I2C%d (%s)"
                                    % (sda, scl, bus_id, label))
            if bus_id in ids_seen:
                problems.append("hardware I2C%d is configured on two different pin pairs" % bus_id)
            ids_seen[bus_id] = True
        addresses = [t["address"] for t in members]
        for addr in addresses:
            if addresses.count(addr) > 1:
                problems.append("two ToF sensors share address 0x%02X on one bus (%s)" % (addr, label))
                break
        if len(members) > 1:
            for t in members:
                if t["xshut"] is None:
                    problems.append("ToF %s shares a bus but has no XSHUT pin" % t["name"])
    return problems
