# ---------------------------------------------------------------------------
# Pins used by the robot for things other than sensors. Sensors must not use
# them. Motors and servos follow the schematic net labels (DIR1, PWM1, DIR2,
# PWM2, ServoPin1, ServoPin2) and match the team's motor.py.
# ---------------------------------------------------------------------------
BOARD_PINS = {
    1: "motor driver 1 #1 PWM (department board, not in the schematic)",  # physical 2
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
# warnings() lists the ones in use.
CHECK_PINS = {
    0: "motor driver 1 #1 DIR on the department board",   # physical 1
    2: "motor driver 1 #2 on the department board",       # physical 4
}

# Things to check on the real wiring (printed by warnings() and test_wiring).
BOARD_CHECKS = (
    "the schematic draws Distance Sensor 1 SDA on the same wire as its VCC; "
    "SDA must go ONLY to physical 4 (GP2) and VCC ONLY to physical 36 (3V3). "
    "With the power off, physical 4 and 36 must NOT be connected (continuity test).",
    "the distance sensors' SDA wires are on physical 1 (GP0) and 4 (GP2), which the department "
    "board may also route to motor driver 1. Nothing may be plugged into motor driver 1: its "
    "inputs would load the I2C lines (no HIGH level -> no readings), and a motor on it could "
    "move with the I2C data.",
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
# Schematic: VCC -> 3V3_OUT, GND -> GND; one sensor per I2C bus.
# Names follow the schematic ("Distance Sensor 1" / "Distance Sensor 2").
# Hardware I2C pin options: I2C0 SDA 0/4/8/12/16/20, SCL 1/5/9/13/17/21
#                           I2C1 SDA 2/6/10/14/18/26, SCL 3/7/11/15/19/27
# Lower I2C_FREQ_HZ to 100000 if the wires are long or the readings are flaky.
# ---------------------------------------------------------------------------
I2C_FREQ_HZ = 100000
TOF_SENSORS = (
    {"name": "dist1", "i2c_id": None, "sda": 2, "scl": 17, "xshut": None, "address": 0x29, "offset_mm": 0},
    # Distance Sensor 2 (REAR, back): SDA GP0 = physical 1, SCL GP11 = physical 15
    {"name": "dist2", "i2c_id": None, "sda": 0, "scl": 11, "xshut": None, "address": 0x29, "offset_mm": 0},
)
TOF_MAX_VALID_MM = 1200        # readings beyond this are reported as "no target" (None)
TOF_MEDIAN_SIZE = 3            # median filter window (odd)
TOF_LONG_RANGE = False         # True -> VCSEL periods 18/14 as in the department example
TOF_PERIOD_MS = 100            # time between two measurements (ms); 0 = back-to-back
TOF_STALE_MS = 300             # a reading older than this is considered stale (longer than TOF_PERIOD_MS)

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
# Measured on the robot.
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
REAR_TOF = "dist2"             # back of the robot (physical 1 / 15): drop slot empty / full
SLOT_READ_MS = 500             # collect readings this long when checking a slot
GRAB_TOF = "dist1"             # front, through the gripper (physical 4 / 22): the box at a pick-up
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

# Not used in the final schematic: GP0-GP3 if motor driver 1 is not used.
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
SERVO_SAFE_RANGE = {"servo1": (155, 164, 174), "servo2": (129, 129, 170)}
# A servo not listed here is reset by going straight to the middle of its
# safe range.
SERVO_RESET = {}
SERVO_RESET_SETTLE_MS = 600
SERVO_RESET_STEP_MS = 20

#   1. drive on slowly until the GRAB distance sensor reads GRAB_STOP_MM or
#      less (GRAB_APPROACH_MAX_MS is a safety limit: then it stops anyway),
#   2. servo1 down to its low end, 3. servo2 down to its low end (gripper
#      closes on the box), 4. servo1 up to its high end,
#   5. go on following the route.
# Drop-off, at the end of the route: servo2 opens by GRIPPER_RELEASE_DEG.
# While driving the gripper is held at GRIPPER_CARRY: servo1 up, servo2 open
# (the high ends); it goes there right after the safe reset at the start.
GRAB_STOP_MM = 50              # stop in front of the box at this GRAB sensor distance (mm)
GRAB_NO_READING_MAX_MS = 500
MIN_MOVING_SPEED = 40
GRAB_APPROACH_SPEED = 50       # motor power (%) while driving up to the box
GRAB_REVERSE_SPEED = 75        # Reverse speed
GRAB_APPROACH_MAX_MS = 1500    # never drive towards the box longer than this
GRIPPER_STEP_MS = 10           # servos turn 1 degree every this many ms (gentle, no jumps)
GRIPPER_PAUSE_MS = 300         # wait after each gripper movement
GRIPPER_LIFT = {"servo1": "high", "servo2": "low"}
GRIPPER_CARRY = {"servo1": "medium", "servo2": "high"}   # pose while driving ("low" / "high" end)
GRIPPER_SAFE = {"servo1": "low", "servo2": "low"}
GRIPPER_RELEASE_DEG = 10       # TODO tune: how far servo2 opens to let go of the box
STOP_AFTER_PICKUP = False

DROP_MOVE_SPEED = 50           # motor power (%) for the small moves at the drop-off
DROP_BACK_MS = 400             # TODO tune: back off this long after letting go
DROP_PUSH_MS = 650             # drive forward this long, pushing the box in
DROP_EXIT_MS = 800             # TODO tune: then back out this long
SERVO_PUSH_ANGLE = {"servo2": 120}   # gripper closed this far for the push only (min 130 - 10)
DROP_DRIVE_MS = 7277
DROP_SERVO1_ANGLE = 173        # arm angle before letting go; kept inside servo1's safe range (155 - 173)

PICKUP_ORDER = ("GREEN_1", "GREEN_2", "RED_1", "RED_2")
PICKUP_FOLLOW_SPEED = 60       # motor power (%) while following the box's line
                               # (steps 1 and 5); 60 = DRIVE_SPEED
PICKUP_STEER = 20              # the wheel on the line's side runs this much
                               # slower (%) - the slow wheel (60 - 20 = 40) must still turn
PICKUP_CROSS_CONFIRM = 2       # all four sensors white this many reads in a row = the cross line
PICKUP_TO_CROSS_MAX_MS = 6000  # step 1 time limit
PICKUP_UTURN_DIRECTION = {"GREEN_1": "right", "GREEN_2": "left", "RED_1": "left", "RED_2": "right"}
PICKUP_REVERSE_MIN_MS = 1400      # the U-turn never ends earlier than this
PICKUP_UTURN_MAX_MS = 6000     # step 4 time limit
PICKUP_BACK_MAX_MS = 8000      # step 5 time limit

# ---------------------------------------------------------------------------
# Backup line follower (control/line_follower.py). Speeds are % of full PWM.
# Start conservative and raise the speed once it follows reliably.
# ---------------------------------------------------------------------------
START_STOP_SENSOR = "left"     # start / stop button: "left" / "right" crash sensor, or "push" = the
                               # push button on PUSH_BUTTON_PIN.
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
        problems.append("START_STOP_SENSOR must be 'left', 'right' or 'push', not %r" % (START_STOP_SENSOR,))
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
    if REAR_TOF not in [t["name"] for t in TOF_SENSORS]:
        problems.append("REAR_TOF %r is not one of TOF_SENSORS" % (REAR_TOF,))
    if GRAB_TOF not in [t["name"] for t in TOF_SENSORS]:
        problems.append("GRAB_TOF %r is not one of TOF_SENSORS" % (GRAB_TOF,))
    if GRAB_TOF == REAR_TOF:
        problems.append("GRAB_TOF and REAR_TOF must be two different sensors")
    if GRAB_REACH_MM >= GRAB_SEE_MM:
        problems.append("GRAB_REACH_MM must be smaller than GRAB_SEE_MM")
    if TOF_LONG_RANGE:
        problems.append("TOF_LONG_RANGE must stay False: libs/VL53L0X.py set_Vcsel_pulse_period() is buggy")
    for name, angle in SERVO_PUSH_ANGLE.items():
        safe = SERVO_SAFE_RANGE.get(name)
        if safe is None or not (0 <= angle <= safe[1]):
            problems.append("SERVO_PUSH_ANGLE[%r] must be between 0 and the top of its safe range" % name)
    push2 = SERVO_PUSH_ANGLE.get("servo2")
    safe2 = SERVO_SAFE_RANGE.get("servo2")
    if push2 is not None and safe2 is not None and push2 < safe2[0] - 10:
        problems.append("SERVO_PUSH_ANGLE['servo2'] must be at least SERVO_SAFE_RANGE['servo2'][0] - 10"
                        " (control/dropoff.py PUSH_MAX_BELOW_SAFE_DEG)")
    if GRAB_NO_READING_MAX_MS <= TOF_STALE_MS:
        problems.append("GRAB_NO_READING_MAX_MS must be longer than TOF_STALE_MS")
    for name in ("DROP_BACK_MS", "DROP_PUSH_MS", "DROP_EXIT_MS", "DROP_DRIVE_MS"):
        if globals()[name] < 0:
            problems.append("%s must not be negative" % name)
    if DROP_DRIVE_MS > 15000:
        problems.append("DROP_DRIVE_MS must be at most 15000 (15 s of driving blind into the branch)")
    safe1 = SERVO_SAFE_RANGE.get("servo1")
    if safe1 is not None and not (safe1[0] <= DROP_SERVO1_ANGLE <= safe1[1]):
        problems.append("DROP_SERVO1_ANGLE must lie inside SERVO_SAFE_RANGE['servo1'] %s" % (safe1,))
    if not 1 <= DROP_MOVE_SPEED <= 100:
        problems.append("DROP_MOVE_SPEED must be 1 - 100 (0 would make Robot.forward use full speed)")
    for name, speed in (("GRAB_APPROACH_SPEED", GRAB_APPROACH_SPEED),
                        ("PICKUP_FOLLOW_SPEED", PICKUP_FOLLOW_SPEED),
                        ("PICKUP_FOLLOW_SPEED - PICKUP_STEER (the slower wheel)", PICKUP_FOLLOW_SPEED - PICKUP_STEER),
                        ("DROP_MOVE_SPEED", DROP_MOVE_SPEED)):
        if speed < MIN_MOVING_SPEED:
            problems.append("%s is %d %%, below MIN_MOVING_SPEED (%d %%): the robot would not move"
                            % (name, speed, MIN_MOVING_SPEED))
    for box, direction in PICKUP_UTURN_DIRECTION.items():
        if direction not in ("left", "right"):
            problems.append("PICKUP_UTURN_DIRECTION[%r] must be 'left' or 'right'" % box)
    if MAG_NORTH_SIGN not in (1, -1):
        problems.append("MAG_NORTH_SIGN must be 1 or -1")
    for name in SERVO_PINS:
        safe = SERVO_SAFE_RANGE.get(name)
        if safe is not None and not (0 <= safe[0] < safe[1] <= 270):
            problems.append("SERVO_SAFE_RANGE[%r] must be (low, high) with 0 <= low < high <= 270" % name)
        path = SERVO_RESET.get(name)
        if path is not None and (safe is None or not (safe[0] <= path[0] <= safe[1] and safe[0] <= path[1] <= safe[1])):
            problems.append("SERVO_RESET[%r] must start and end inside SERVO_SAFE_RANGE[%r]" % (name, name))
        if GRIPPER_CARRY.get(name, "high") not in ("low", "high"):
            problems.append("GRIPPER_CARRY[%r] must be 'low' or 'high'" % name)
    return problems


def sensor_pins():
    """{GP: description} for every pin the sensors use."""
    pins = {
        LINE_OUTER_LEFT_PIN: "line outer-left", LINE_INNER_LEFT_PIN: "line inner-left",
        LINE_INNER_RIGHT_PIN: "line inner-right", LINE_OUTER_RIGHT_PIN: "line outer-right",
        CRASH_LEFT_PIN: "crash left", CRASH_RIGHT_PIN: "crash right",
        IR_RX_PIN: "IR receiver", MAG_PIN: "magnetic sensor",
        PUSH_BUTTON_PIN: "push button",
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
