"""
Sensor configuration: every sensor pin number and tuning constant lives here.

Pin numbers in code are Pico GPIO numbers ("GPn"). Our board reports itself as
a Raspberry Pi Pico 2 (RP2350); its header pinout is identical to the Pico W
shown on Moodle, so all pin numbers below apply to both. The jumper
headers on the department robot board are labelled with the PHYSICAL pin
numbers (1-40) instead, e.g. GP18 is "24 on the jumper" and GP28 is "34".
The comment after each pin gives both; physical_pin() converts. Confirm every
value with the electrical team before powering up, and run validate() once to
catch pin clashes.
"""

# ---------------------------------------------------------------------------
# Pins already wired on the department robot board (from the department
# example code). Sensors must not use them. Delete an entry if your team
# really does not use that header.
# ---------------------------------------------------------------------------
BOARD_PINS = {
    0: "motor driver 1 #1 DIR",       # physical 1
    1: "motor driver 1 #1 PWM",       # physical 2
    2: "motor driver 1 #2 (assumed)",  # physical 4
    3: "motor driver 1 #2 (assumed)",  # physical 5
    4: "motor driver 2 #1 DIR",       # physical 6
    5: "motor driver 2 #1 PWM",       # physical 7
    6: "motor driver 2 #2 PWM",       # physical 9
    7: "motor driver 2 #2 DIR",       # physical 10
    13: "servo 2 header (assumed)",   # physical 17
    15: "servo 1 header",             # physical 20
}

# ---------------------------------------------------------------------------
# Line tracking sensors: 4 x DFRobot SEN0017 (digital, white -> 1, black -> 0).
# Power them from 3V3(OUT) (physical pin 36) so their output is safe for the Pico.
# Listed left to right across the robot.
# ---------------------------------------------------------------------------
LINE_OUTER_LEFT_PIN = 18       # GP18 = physical 24
LINE_INNER_LEFT_PIN = 19       # GP19 = physical 25
LINE_INNER_RIGHT_PIN = 20      # GP20 = physical 26
LINE_OUTER_RIGHT_PIN = 21      # GP21 = physical 27
LINE_WHITE_IS_HIGH = True
LINE_SAMPLES_PER_READ = 3      # majority vote over this many pin reads (use an odd number)
JUNCTION_CONFIRM_SAMPLES = 2   # an outer sensor must see white this many updates in a row
JUNCTION_CLEAR_SAMPLES = 3     # both outer sensors black this many updates to end a junction
LINE_LOST_MS = 150             # all four sensors black for this long -> line lost / end of line

# ---------------------------------------------------------------------------
# Time-of-flight distance sensors: 2 x VL53L0X (I2C). Power from 3V3(OUT), not 5V.
# Default: one sensor per hardware I2C bus, so both keep address 0x29.
# I2C0 on GP8/GP9 is what the department examples use.
# Hardware I2C pin options: I2C0 SDA 0/4/8/12/16/20, SCL 1/5/9/13/17/21
#                           I2C1 SDA 2/6/10/14/18/26, SCL 3/7/11/15/19/27
# SDA and SCL must belong to the same bus (e.g. GP8 + GP9 for I2C0).
# Set "i2c_id" to None to use SoftI2C on any two free pins.
# Alternative wiring (both sensors on ONE bus): give every sensor an "xshut" pin
# and a distinct "address" (e.g. 0x29 and 0x30).
# Lower I2C_FREQ_HZ to 100000 if the wires are long or the readings are flaky.
# ---------------------------------------------------------------------------
I2C_FREQ_HZ = 400000
TOF_SENSORS = (
    # front: SDA GP8 = physical 11, SCL GP9 = physical 12
    {"name": "front", "i2c_id": 0, "sda": 8, "scl": 9, "xshut": None, "address": 0x29, "offset_mm": 0},
    # side: SDA GP10 = physical 14, SCL GP11 = physical 15
    {"name": "side", "i2c_id": 1, "sda": 10, "scl": 11, "xshut": None, "address": 0x29, "offset_mm": 0},
)
TOF_MAX_VALID_MM = 1200        # readings beyond this are reported as "no target" (None)
TOF_MEDIAN_SIZE = 3            # median filter window (odd)
TOF_LONG_RANGE = False         # True -> VCSEL periods 18/14 as in the department example
TOF_STALE_MS = 200             # a reading older than this is considered stale

# ---------------------------------------------------------------------------
# Crash / bump sensors: 2 x DFRobot SEN0138 (micro-switch board, 5V supply ONLY).
# Output: HIGH (5V) when released, LOW when pressed.
# Take 5V from the robot board's 5V rail (VBUS, physical 40, is only 5V while
# USB is plugged in).
# !!! Put a voltage divider (e.g. 10k series, 20k to GND) between the sensor
# !!! output and the Pico pin. RP2040 (Pico W) GPIOs are NOT 5V tolerant, and the
# !!! RP2350 (Pico 2) only tolerates 5V inputs while it is powered, so always divide.
# ---------------------------------------------------------------------------
CRASH_LEFT_PIN = 16            # GP16 = physical 21
CRASH_RIGHT_PIN = 17           # GP17 = physical 22
CRASH_ACTIVE_LOW = True        # pressed -> 0V
CRASH_DEBOUNCE_MS = 15

# ---------------------------------------------------------------------------
# IR receiver: 1 x Vishay TSOP4838 (38 kHz demodulator; output LOW while an IR
# burst is being received). Pins: 1 = OUT, 2 = GND, 3 = VS. Power VS from
# 3V3(OUT) so the output (internal 30k pull-up to VS) stays at 3.3V.
# ---------------------------------------------------------------------------
IR_RX_PIN = 22                 # GP22 = physical 29
IR_FRAME_GAP_US = 8000         # silence longer than this ends a frame
IR_PRESENT_WINDOW_MS = 150     # signal_present(): any IR activity within this window

# ---------------------------------------------------------------------------
# Magnetic (Hall effect) sensor: 1 x 3-wire module.
# VCC -> 3V3(OUT) (physical 36), GND -> GND, signal -> MAG_PIN (an ADC pin).
# Measured on the robot (2026-10-13): ANALOG (linear) output. Zero field reads
# about 2.1-2.25 V and drifts by about 100 mV; a magnet moves it either way
# (one pole up, the other down) and a close magnet saturates it at about
# 0.05 V / 3.0 V. The thresholds below sit above the drift and well below the
# 400-2000 mV change of a nearby magnet.
# ---------------------------------------------------------------------------
MAG_PIN = 26                   # GP26 = physical 31 (ADC0)
MAG_MODE = "analog"            # "digital" (output only 0/1) or "analog" (voltage follows the field)
MAG_ACTIVE_LOW = True          # digital modules only: output goes LOW while a magnet is detected
MAG_DEBOUNCE_MS = 10           # digital modules only
MAG_ON_MV = 300                # analog: |field| at or above this -> magnet detected
MAG_OFF_MV = 200               # analog: |field| at or below this -> magnet gone (hysteresis)

# Still free after the above: GP12 (16), GP14 (19), GP27 (32), GP28 (34).
# GP26-GP28 are the only analogue (ADC) inputs - keep them for analogue sensors.
# The department examples put a start button on GP12 and an LED on GP28.
# The on-board LED is not a header pin: use Pin("LED") (GP25 on Pico 2, wireless chip on Pico W).


# ---------------------------------------------------------------------------
# Pin helpers and sanity check
# ---------------------------------------------------------------------------
# GPn -> physical header pin number (Pico / Pico W / Pico 2, top view, pin 1 next to the USB socket).
_PHYSICAL = {
    0: 1, 1: 2, 2: 4, 3: 5, 4: 6, 5: 7, 6: 9, 7: 10, 8: 11, 9: 12,
    10: 14, 11: 15, 12: 16, 13: 17, 14: 19, 15: 20, 16: 21, 17: 22, 18: 24, 19: 25,
    20: 26, 21: 27, 22: 29, 26: 31, 27: 32, 28: 34,
}
_I2C_PINS = {
    0: ((0, 4, 8, 12, 16, 20), (1, 5, 9, 13, 17, 21)),
    1: ((2, 6, 10, 14, 18, 26), (3, 7, 11, 15, 19, 27)),
}


def physical_pin(gpio):
    """Physical header pin (the number on the robot board's jumpers) for GPn, or None."""
    return _PHYSICAL.get(gpio)


def _label(gpio):
    phys = _PHYSICAL.get(gpio)
    if phys is None:
        return "GP%d" % gpio
    return "GP%d (physical %d)" % (gpio, phys)


def validate():
    """Return a list of human-readable configuration problems (empty if OK)."""
    problems = []
    used = {}
    for gpio, what in BOARD_PINS.items():
        used[gpio] = "board: " + what

    def claim(pin, what):
        if pin is None:
            return
        if pin not in _PHYSICAL:
            problems.append("GP%d (%s) is not a usable header GPIO" % (pin, what))
        if pin in used:
            problems.append("%s is used by both %s and %s" % (_label(pin), used[pin], what))
        else:
            used[pin] = what

    claim(LINE_OUTER_LEFT_PIN, "line outer-left")
    claim(LINE_INNER_LEFT_PIN, "line inner-left")
    claim(LINE_INNER_RIGHT_PIN, "line inner-right")
    claim(LINE_OUTER_RIGHT_PIN, "line outer-right")
    claim(CRASH_LEFT_PIN, "crash left")
    claim(CRASH_RIGHT_PIN, "crash right")
    claim(IR_RX_PIN, "IR receiver")
    claim(MAG_PIN, "magnetic sensor")
    if MAG_MODE not in ("digital", "analog"):
        problems.append("MAG_MODE must be 'digital' or 'analog', not %r" % (MAG_MODE,))
    elif MAG_MODE == "analog" and MAG_PIN not in (26, 27, 28):
        problems.append("analog magnetic sensor needs an ADC pin (GP26/27/28), not GP%d" % MAG_PIN)

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
