"""
Sensor configuration: every sensor pin number and tuning constant lives here.

The pins follow the team schematic "SCH_Schematic1" V1.0 (EasyEDA, updated
2026-10-13), except two signals moved on the robot on 2026-10-14: the crash
sensors from GP6/GP7 to GP8/GP9 (physical 11/12) and the IR receiver from GP15
to GP14 (physical 19). Change pins only together with the schematic.

Pin numbers in code are Pico GPIO numbers ("GPn"). The board is a Raspberry Pi
Pico 2 (RP2350); its header pinout is identical to the Pico W shown on Moodle.
The jumper headers on the department robot board are labelled with the
PHYSICAL pin numbers (1-40) instead, e.g. GP18 is "24 on the jumper". The
comment after each pin gives both; physical_pin() converts. Run validate()
(hard errors) and warnings() (things to check on the board) after any change.
"""

# ---------------------------------------------------------------------------
# Pins wired on the robot board to the motor drivers (department example code
# and the team's motor.py: motor 3 = DIR GP4 / PWM GP5, motor 4 = DIR GP7 /
# PWM GP6). Sensors must not use them: a sensor output on a motor driver
# input could switch the motor on.
# ---------------------------------------------------------------------------
BOARD_PINS = {
    0: "motor driver 1 #1 DIR",               # physical 1
    1: "motor driver 1 #1 PWM",               # physical 2
    2: "motor driver 1 #2 (assumed)",          # physical 4
    3: "motor driver 1 #2 (assumed)",          # physical 5
    4: "motor 3 DIR (motor driver 2 #1)",     # physical 6
    5: "motor 3 PWM (motor driver 2 #1)",     # physical 7
    6: "motor 4 PWM (motor driver 2 #2)",     # physical 9
    7: "motor 4 DIR (motor driver 2 #2)",     # physical 10
}

# Pins the department examples used for something else on the robot board.
# Sensors may use them, but then check on the board that nothing else is
# connected (warnings() lists the ones in use).
CHECK_PINS = {
    13: "servo 2 header (assumed)",                      # physical 17
    15: "servo 1 header in the department example",      # physical 20
}

# ---------------------------------------------------------------------------
# Line tracking sensors: 4 x DFRobot SEN0017 (digital, white -> 1, black -> 0).
# Schematic: VCC -> 3V3_OUT (physical 36), GND -> GND (physical 38).
# Listed left to right across the robot (schematic names in brackets).
# ---------------------------------------------------------------------------
LINE_OUTER_LEFT_PIN = 20       # GP20 = physical 26  (Line Sensor "Edge Left")
LINE_INNER_LEFT_PIN = 21       # GP21 = physical 27  (Line Sensor "Centre Left")
LINE_INNER_RIGHT_PIN = 19      # GP19 = physical 25  (Line Sensor "Centre Right")
LINE_OUTER_RIGHT_PIN = 18      # GP18 = physical 24  (Line Sensor "Edge Right")
LINE_WHITE_IS_HIGH = True
LINE_SAMPLES_PER_READ = 3      # majority vote over this many pin reads (use an odd number)
JUNCTION_CONFIRM_SAMPLES = 2   # an outer sensor must see white this many updates in a row
JUNCTION_CLEAR_SAMPLES = 3     # both outer sensors black this many updates to end a junction
LINE_LOST_MS = 150             # all four sensors black for this long -> line lost / end of line

# ---------------------------------------------------------------------------
# Time-of-flight distance sensors: 2 x VL53L0X (I2C, boards labelled + - C D).
# Schematic: VCC -> 3V3_OUT, GND -> GND; one sensor per hardware I2C bus, so
# both keep address 0x29 (the boards have no XSHUT pin).
# Names follow the schematic ("Distance Sensor 1" / "Distance Sensor 2").
# Hardware I2C pin options: I2C0 SDA 0/4/8/12/16/20, SCL 1/5/9/13/17/21
#                           I2C1 SDA 2/6/10/14/18/26, SCL 3/7/11/15/19/27
# Lower I2C_FREQ_HZ to 100000 if the wires are long or the readings are flaky.
# ---------------------------------------------------------------------------
I2C_FREQ_HZ = 400000
TOF_SENSORS = (
    # Distance Sensor 1: SDA (D) GP12 = physical 16, SCL (C) GP13 = physical 17, I2C0
    {"name": "dist1", "i2c_id": 0, "sda": 12, "scl": 13, "xshut": None, "address": 0x29, "offset_mm": 0},
    # Distance Sensor 2: SDA (D) GP10 = physical 14, SCL (C) GP11 = physical 15, I2C1
    {"name": "dist2", "i2c_id": 1, "sda": 10, "scl": 11, "xshut": None, "address": 0x29, "offset_mm": 0},
)
TOF_MAX_VALID_MM = 1200        # readings beyond this are reported as "no target" (None)
TOF_MEDIAN_SIZE = 3            # median filter window (odd)
TOF_LONG_RANGE = False         # True -> VCSEL periods 18/14 as in the department example
TOF_STALE_MS = 200             # a reading older than this is considered stale

# ---------------------------------------------------------------------------
# Crash / bump sensors: 2 x DFRobot SEN0138 (micro-switch with on-board pull-up).
# Output: HIGH when released, LOW when pressed.
# Schematic: VCC -> 3V3_OUT, so the output is at most 3.3V and goes straight to
# the Pico pin, no divider. (DFRobot rates the board for 5V; the switch and
# pull-up work at 3.3V, only the LED is dimmer.) If the sensors are ever
# powered from 5V instead, a divider (10k series, 20k to GND) is REQUIRED.
# Signal pins moved on 2026-10-14 from GP6/GP7 (schematic) to GP8/GP9, because
# GP6/GP7 drive motor 4.
# ---------------------------------------------------------------------------
CRASH_LEFT_PIN = 8             # GP8 = physical 11
CRASH_RIGHT_PIN = 9            # GP9 = physical 12
CRASH_ACTIVE_LOW = True        # pressed -> 0V
CRASH_DEBOUNCE_MS = 15

# ---------------------------------------------------------------------------
# IR receiver: 1 x Vishay TSOP4838 (38 kHz demodulator; output LOW while an IR
# burst is being received). Pins: 1 = OUT, 2 = GND, 3 = VS.
# Schematic: VS -> 3V3_OUT, so the output (internal 30k pull-up to VS) stays at 3.3V.
# Signal pin moved on 2026-10-14 from GP15 (schematic, also the servo 1 header)
# to GP14.
# ---------------------------------------------------------------------------
IR_RX_PIN = 14                 # GP14 = physical 19
IR_FRAME_GAP_US = 8000         # silence longer than this ends a frame
IR_PRESENT_WINDOW_MS = 150     # signal_present(): any IR activity within this window

# ---------------------------------------------------------------------------
# Magnetic (Hall effect) sensor ("Magnetometer" in the schematic), 3 wires:
# 1 OUT -> MAG_PIN (an ADC pin), 2 GND -> GND, 3 VS -> 3V3_OUT.
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

# Not used by any sensor or motor: GP15 (20, servo 1 header), GP16, GP17
# (21, 22), GP22 (29), GP27 (32), GP28 (34). GP26-GP28 are the only analogue
# (ADC) inputs.
# The servos and the start button are not in the schematic yet; pick from these.
# The on-board LED is not a header pin: use Pin("LED") (GP25 on Pico 2).


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


def sensor_pins():
    """{GP: description} for every pin the sensors use."""
    pins = {
        LINE_OUTER_LEFT_PIN: "line outer-left", LINE_INNER_LEFT_PIN: "line inner-left",
        LINE_INNER_RIGHT_PIN: "line inner-right", LINE_OUTER_RIGHT_PIN: "line outer-right",
        CRASH_LEFT_PIN: "crash left", CRASH_RIGHT_PIN: "crash right",
        IR_RX_PIN: "IR receiver", MAG_PIN: "magnetic sensor",
    }
    for tof in TOF_SENSORS:
        pins[tof["sda"]] = "ToF %s SDA" % tof["name"]
        pins[tof["scl"]] = "ToF %s SCL" % tof["name"]
        if tof["xshut"] is not None:
            pins[tof["xshut"]] = "ToF %s XSHUT" % tof["name"]
    return pins


def warnings():
    """Things to check on the robot board (not errors): sensors on CHECK_PINS."""
    out = []
    for gp, what in sorted(sensor_pins().items()):
        if gp in CHECK_PINS:
            out.append("%s (%s) is also %s - make sure nothing else is connected to it"
                       % (_label(gp), what, CHECK_PINS[gp]))
    return out
