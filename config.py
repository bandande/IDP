"""
Pin configuration: every sensor pin, the robot's other pins and the sensor
tuning constants live here.

The pins follow the FINAL team schematic "SCH_Schematic1" (EasyEDA, file
SCH_Schematic1_2026-10-15.pdf, page "sensor_schematic_1"). Change them only
together with the schematic.

Pin numbers in code are Pico GPIO numbers ("GPn"). The board is a Raspberry Pi
Pico 2 (RP2350); its header pinout is identical to the Pico W shown on Moodle.
The jumper headers on the robot board and the schematic use PHYSICAL pin
numbers (1-40) instead, e.g. GP18 is physical pin 24. The comment after each
pin gives both; physical_pin() converts. Run validate() (errors) and
warnings() (things to check on the board) after any change.
"""

# ---------------------------------------------------------------------------
# Pins used by the robot for things other than sensors. Sensors must not use
# them. Motors and servos follow the schematic net labels (DIR1, PWM1, DIR2,
# PWM2, ServoPin1, ServoPin2) and match the team's motor.py.
# ---------------------------------------------------------------------------
BOARD_PINS = {
    0: "motor driver 1 #1 DIR (department board, not in the schematic)",  # physical 1
    1: "motor driver 1 #1 PWM (department board, not in the schematic)",  # physical 2
    2: "motor driver 1 #2 (assumed, not in the schematic)",               # physical 4
    3: "motor driver 1 #2 (assumed, not in the schematic)",               # physical 5
    4: "motor 1 DIR (DIR1)",                                              # physical 6
    5: "motor 1 PWM (PWM1)",                                              # physical 7
    6: "motor 2 PWM (PWM2)",                                              # physical 9
    7: "motor 2 DIR (DIR2)",                                              # physical 10
    13: "servo 2 signal (ServoPin2)",                                     # physical 17
    15: "servo 1 signal (ServoPin1)",                                     # physical 20
}

# Indicator LEDs, driven through a 4-channel optocoupler (220 ohm on both
# sides): GPIO HIGH -> LED on. Outputs, so sensors must not use these pins.
LED_PINS = {
    "yellow": 22,                  # GP22 = physical 29 -> optocoupler ch1 -> Yellow_LED
    "red": 27,                     # GP27 = physical 32 -> optocoupler ch2 -> Red_LED
    "green": 28,                   # GP28 = physical 34 -> optocoupler ch3 -> Green_LED
}

# Pins used by sensors although they have another use on the robot board.
# warnings() lists the ones in use. Empty with the final schematic.
CHECK_PINS = {}

# Things to check on the real wiring (printed by warnings() and test_wiring).
BOARD_CHECKS = (
    "the schematic draws Distance Sensor 1 SDA on the same wire as its VCC; "
    "SDA must go ONLY to physical 21 (GP16) and VCC ONLY to physical 36 (3V3). "
    "With the power off, physical 21 and 36 must NOT be connected (continuity test).",
)

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
# both keep address 0x29 (the boards have no XSHUT pin). See BOARD_CHECKS
# about Distance Sensor 1's SDA wire.
# Names follow the schematic ("Distance Sensor 1" / "Distance Sensor 2").
# Hardware I2C pin options: I2C0 SDA 0/4/8/12/16/20, SCL 1/5/9/13/17/21
#                           I2C1 SDA 2/6/10/14/18/26, SCL 3/7/11/15/19/27
# Lower I2C_FREQ_HZ to 100000 if the wires are long or the readings are flaky.
# ---------------------------------------------------------------------------
I2C_FREQ_HZ = 400000
TOF_SENSORS = (
    # Distance Sensor 1: SDA (D) GP16 = physical 21, SCL (C) GP17 = physical 22, I2C0
    {"name": "dist1", "i2c_id": 0, "sda": 16, "scl": 17, "xshut": None, "address": 0x29, "offset_mm": 0},
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
# ---------------------------------------------------------------------------
CRASH_LEFT_PIN = 9             # GP9 = physical 12  (Crash Sensor L DIG/S)
CRASH_RIGHT_PIN = 8            # GP8 = physical 11  (Crash Sensor R DIG/S)
CRASH_ACTIVE_LOW = True        # pressed -> 0V
CRASH_DEBOUNCE_MS = 15
# ---------------------------------------------------------------------------
PUSH_BUTTON_PIN = 12           # GP12 = physical 16 (Push Button DIG/S)
# ---------------------------------------------------------------------------
# IR receiver: 1 x Vishay TSOP4838 (38 kHz demodulator; output LOW while an IR
# burst is being received). Pins: 1 = OUT, 2 = GND, 3 = VS.
# Schematic: VS -> 3V3_OUT, so the output (internal 30k pull-up to VS) stays at 3.3V.
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

# ---------------------------------------------------------------------------
# Task sensors (control/task_sensors.py, used by main.py). Values marked
# TODO are placeholders until they are measured / the task rule is known.
# ---------------------------------------------------------------------------
# Magnet pole of the box at pick-up -> rack level.
MAG_READ_MS = 400              # watch the magnet sensor this long at a pick-up
MAG_NORTH_SIGN = 1             # TODO measure: sign of the field (+1 / -1) when a NORTH pole faces the sensor
MAG_POLE_LEVEL = {"north": "TOP", "south": "BOTTOM"}   # TODO: the task rule (pole -> rack level)
MAG_NO_POLE_LEVEL = "BOTTOM"   # level used when no magnet is found
# Front distance sensor: is the drop slot already full?
FRONT_TOF = "dist1"            # TODO check: which of TOF_SENSORS faces forward
SLOT_READ_MS = 250             # collect readings this long in front of a slot
SLOT_OCCUPIED_MM = 170         # TODO measure: the robot looks into a slot from its junction (turned
                               # to face it, before driving in). A box anywhere in the slot - also one
                               # put in from the other side of the rack - is closer than this; an
                               # empty slot reads further (or nothing). Measure both on the table.
# Grab distance sensor (the second ToF): how far away the box to pick up is,
# for aiming at it and driving up to it (TaskSensors.box_distance_now() etc.).
# How it is used is still being decided. The starting values are the ones in
# main.py's grabbing draft (DISTANCE_TO_SCAN_PICK_UP / DISTANCE_TO_PICK_UP).
GRAB_TOF = "dist2"             # TODO check: which of TOF_SENSORS is the grab / aiming sensor
GRAB_READ_MS = 150             # box_distance(): collect readings this long (robot standing still)
GRAB_SEE_MM = 150              # TODO measure: closer than this = a box is in front of the sensor
GRAB_REACH_MM = 20             # TODO measure: this close or closer = close enough to grab. The VL53L0X
                               # is not reliable below ~30 mm: check the real reading at grab distance.
# Traffic light (IR receiver). Before driving along a line listed here (pairs
# of node names from MAP.py), the robot stops and waits for green.
# The light is at the ramp entrance: the robot waits at START before driving
# north up the ramp (START -> TOP_CENTRE). Not on the way back down the ramp.
TRAFFIC_LIGHT_EDGES = (("START", "TOP_CENTRE"),)
TRAFFIC_RED_WHEN_IR = False    # IR beacon active means GREEN (go); no IR = red (wait)
TRAFFIC_IR_CONFIRM_MS = 200    # IR = green: IR must be seen without a break this long (a single
                               # stray pulse only counts for IR_PRESENT_WINDOW_MS = 150 ms)
TRAFFIC_GREEN_AFTER_MS = 1500  # IR = red only: no IR for this long = green (longer than the beacon's off phase)
TRAFFIC_WAIT_MAX_MS = 0        # give up waiting and go after this long (0 = wait forever)

# Not used in the final schematic: GP12 (physical 16; the department examples
# put a start button there), plus GP0-GP3 if motor driver 1 is not used.
# The on-board LED is not a header pin: use Pin("LED") (GP25 on Pico 2).

# ---------------------------------------------------------------------------
# Drive motors (schematic net labels; same pins as the team's motor.py).
# Department motor driver: DIR pin + PWM pin per motor, DIR = 0 is "forward".
# Run tests/test_motors.py first: if the wrong wheel moves in its "left" step,
# swap LEFT and RIGHT here; if a wheel turns backwards, flip its INVERT flag.
# ---------------------------------------------------------------------------
MOTOR_LEFT_DIR_PIN = 4         # DIR1, GP4 = physical 6
MOTOR_LEFT_PWM_PIN = 5         # PWM1, GP5 = physical 7
MOTOR_RIGHT_DIR_PIN = 7        # DIR2, GP7 = physical 10
MOTOR_RIGHT_PWM_PIN = 6        # PWM2, GP6 = physical 9
MOTOR_LEFT_INVERT = False
MOTOR_RIGHT_INVERT = False
MOTOR_PWM_FREQ_HZ = 1000
MOTOR_MIN_DUTY = 0             # % where the wheels just start to turn (tests/test_motors.py ramp)

# Gripper servos (actuators/servo.py; schematic ServoPin1 / ServoPin2).
SERVO_PINS = {"servo1": 15, "servo2": 13}   # GP15 = physical 20, GP13 = physical 17

# ---------------------------------------------------------------------------
# Backup line follower (control/line_follower.py). Speeds are % of full PWM.
# Start conservative and raise the speed once it follows reliably.
# ---------------------------------------------------------------------------
START_STOP_SENSOR = "push"     # crash sensor used as the start / stop button: "left" or "right"
START_DELAY_MS = 1000          # wait after the start press, so the hand is clear of the robot
LOOP_PERIOD_MS = 5             # main loop period
FOLLOW_BASE_SPEED = 35         # forward speed on a straight line
FOLLOW_KP = 35.0               # speed difference per unit of line error (error is -1 .. +1)
FOLLOW_KD = 0.0                # damping: speed difference per (error change per second)
FOLLOW_MAX_CORRECTION = 45     # limit of the speed difference
FOLLOW_SEARCH_SPEED = 30       # spin speed while searching for a lost line
FOLLOW_SEARCH_TIMEOUT_MS = 2000  # stop if the line is not found again within this time
TURN_SPEED = 35                # spin speed for 90 degree turns at junctions
TURN_FORWARD_MS = 300          # drive on after the junction so the wheels are over it, then spin
TURN_TIMEOUT_MS = 3000         # stop if a spin turn does not find the new line
T_STEM_ACTION = "stop"         # line ends in a T (left and right only): "stop", "L" or "R"


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
    claim(PUSH_BUTTON_PIN, "push button")
    claim(IR_RX_PIN, "IR receiver")
    claim(MAG_PIN, "magnetic sensor")
    for colour, pin in LED_PINS.items():
        claim(pin, colour + " LED")
    for name, pin in (("MOTOR_LEFT_DIR_PIN", MOTOR_LEFT_DIR_PIN), ("MOTOR_LEFT_PWM_PIN", MOTOR_LEFT_PWM_PIN),
                      ("MOTOR_RIGHT_DIR_PIN", MOTOR_RIGHT_DIR_PIN), ("MOTOR_RIGHT_PWM_PIN", MOTOR_RIGHT_PWM_PIN)):
        if not BOARD_PINS.get(pin, "").startswith("motor"):
            problems.append("%s = GP%d is not a motor pin in BOARD_PINS" % (name, pin))
    if START_STOP_SENSOR not in ("left", "right", "push"):
        problems.append("START_STOP_SENSOR must be 'left', 'right', or 'push', not %r" % (START_STOP_SENSOR,))
    if T_STEM_ACTION not in ("stop", "L", "R"):
        problems.append("T_STEM_ACTION must be 'stop', 'L' or 'R', not %r" % (T_STEM_ACTION,))
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
    # task sensors
    if FRONT_TOF not in [t["name"] for t in TOF_SENSORS]:
        problems.append("FRONT_TOF %r is not one of TOF_SENSORS" % (FRONT_TOF,))
    if GRAB_TOF not in [t["name"] for t in TOF_SENSORS]:
        problems.append("GRAB_TOF %r is not one of TOF_SENSORS" % (GRAB_TOF,))
    if GRAB_TOF == FRONT_TOF:
        problems.append("GRAB_TOF and FRONT_TOF must be two different sensors")
    if GRAB_REACH_MM >= GRAB_SEE_MM:
        problems.append("GRAB_REACH_MM must be smaller than GRAB_SEE_MM")
    if MAG_NORTH_SIGN not in (1, -1):
        problems.append("MAG_NORTH_SIGN must be 1 or -1")
    for pole in ("north", "south"):
        if MAG_POLE_LEVEL.get(pole) not in ("TOP", "BOTTOM"):
            problems.append("MAG_POLE_LEVEL[%r] must be 'TOP' or 'BOTTOM'" % pole)
    if MAG_NO_POLE_LEVEL not in ("TOP", "BOTTOM"):
        problems.append("MAG_NO_POLE_LEVEL must be 'TOP' or 'BOTTOM'")
    return problems


def sensor_pins():
    """{GP: description} for every pin the sensors use."""
    pins = {
        LINE_OUTER_LEFT_PIN: "line outer-left", LINE_INNER_LEFT_PIN: "line inner-left",
        LINE_INNER_RIGHT_PIN: "line inner-right", LINE_OUTER_RIGHT_PIN: "line outer-right",
        CRASH_LEFT_PIN: "crash left", CRASH_RIGHT_PIN: "crash right",
        IR_RX_PIN: "IR receiver", MAG_PIN: "magnetic sensor", PUSH_BUTTON_PIN: "push button"
    }
    for tof in TOF_SENSORS:
        pins[tof["sda"]] = "ToF %s SDA" % tof["name"]
        pins[tof["scl"]] = "ToF %s SCL" % tof["name"]
        if tof["xshut"] is not None:
            pins[tof["xshut"]] = "ToF %s XSHUT" % tof["name"]
    return pins


def warnings():
    """Things to check on the robot board (not errors): BOARD_CHECKS and sensors on CHECK_PINS."""
    out = list(BOARD_CHECKS)
    for gp, what in sorted(sensor_pins().items()):
        if gp in CHECK_PINS:
            out.append("%s (%s) is also %s - make sure nothing else is connected to it"
                       % (_label(gp), what, CHECK_PINS[gp]))
    return out
