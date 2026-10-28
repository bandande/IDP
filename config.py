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
    # CHANGED (2026-10-27): GP0 (physical 1) and GP2 (physical 4) are now the
    # SDA lines of the two distance sensors (see TOF_SENSORS), so they are no
    # longer listed here. WARNING: if the department motor board really
    # routes GP0 / GP2 to motor driver 1's inputs, those inputs also hang on
    # the I2C lines and can disturb them - nothing must be plugged into
    # motor driver 1.
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
# CHANGED (2026-10-27): GP0 / GP2 are now the distance sensors' SDA lines
# (see TOF_SENSORS) but may also be wired to motor driver 1 on the
# department board, so warnings() / tests/test_wiring.py remind about them.
CHECK_PINS = {
    0: "motor driver 1 #1 DIR on the department board",   # physical 1
    2: "motor driver 1 #2 on the department board",       # physical 4
}

# Things to check on the real wiring (printed by warnings() and test_wiring).
# CHANGED (2026-10-27): Distance Sensor 1's SDA moved from physical 21 to
# physical 4 (GP2); the second check is about motor driver 1 (GP0 - GP3).
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
# Schematic: VCC -> 3V3_OUT, GND -> GND; one sensor per I2C bus (CHANGED
# 2026-10-27: now two SoftI2C buses, see TOF_SENSORS), so both keep address
# 0x29 (the boards have no XSHUT pin). See BOARD_CHECKS about the SDA wires.
# Names follow the schematic ("Distance Sensor 1" / "Distance Sensor 2").
# Hardware I2C pin options: I2C0 SDA 0/4/8/12/16/20, SCL 1/5/9/13/17/21
#                           I2C1 SDA 2/6/10/14/18/26, SCL 3/7/11/15/19/27
# Lower I2C_FREQ_HZ to 100000 if the wires are long or the readings are flaky.
# CHANGED (2026-10-27): 100000 (was 400000) - a sensor showed thousands of
# I2C errors; the slower clock is more tolerant of long / loose wires.
# ---------------------------------------------------------------------------
I2C_FREQ_HZ = 100000
TOF_SENSORS = (
    # CHANGED (2026-10-27): re-wired by the team - each SDA wire moved
    # ("21 to 4", "14 to 1"), the SCL wires stayed. The new pairs are not
    # hardware I2C pairs (GP2 is an I2C1 SDA pin but GP17 an I2C0 SCL pin;
    # GP0 is I2C0 SDA but GP11 I2C1 SCL), so "i2c_id": None = SoftI2C (a
    # software I2C on any two pins; open-drain like the hardware one, safe).
    # To use hardware I2C again, also move each SCL: dist1 SCL to physical 5
    # (GP3, then i2c_id 1), dist2 SCL to physical 2 (GP1, then i2c_id 0).
    # Distance Sensor 1 (GRAB, front): SDA GP2 = physical 4, SCL GP17 = physical 22
    {"name": "dist1", "i2c_id": None, "sda": 2, "scl": 17, "xshut": None, "address": 0x29, "offset_mm": 0},
    # Distance Sensor 2 (REAR, back): SDA GP0 = physical 1, SCL GP11 = physical 15
    {"name": "dist2", "i2c_id": None, "sda": 0, "scl": 11, "xshut": None, "address": 0x29, "offset_mm": 0},
)
TOF_MAX_VALID_MM = 1200        # readings beyond this are reported as "no target" (None)
TOF_MEDIAN_SIZE = 3            # median filter window (odd)
TOF_LONG_RANGE = False         # True -> VCSEL periods 18/14 as in the department example
                               # CHANGED (2026-10-27): keep False - the driver code for it
                               # (libs/VL53L0X.py set_Vcsel_pulse_period) has register-width
                               # bugs; validate() reports True as a problem.
# CHANGED (2026-10-27): the sensors no longer measure back-to-back (one
# measurement straight after the other, ~30 per second, laser almost always
# on). The teammate suspects heat made them fail, so each now takes ONE
# measurement every TOF_PERIOD_MS ("timed" mode of the VL53L0X): about 10 per
# second, the laser is off most of the time. poll() also checks the sensor
# over I2C only every TOF_PERIOD_MS / 2 instead of every loop.
# At the pick-up approach (~6 cm/s) 10 readings per second is one reading
# every ~6 mm, enough to stop near GRAB_STOP_MM.
TOF_PERIOD_MS = 100            # time between two measurements (ms); 0 = back-to-back as before
TOF_STALE_MS = 300             # a reading older than this is considered stale (CHANGED: was 200,
                               # it must be longer than TOF_PERIOD_MS)

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
# ADOPTED (2026-10-23) from the partner's main (91d3af5): a separate push
# button on the free pin GP12. Wired like the crash sensors: pressed -> 0V
# (the Pico's internal pull-up holds it HIGH when released). It is the start /
# stop button only when START_STOP_SENSOR = "push" (below).
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
# CHANGED (2026-10-27): the two distance sensors' jobs (the team's design).
# The robot was re-wired the same day (twice; the latest wiring is below and
# in TOF_SENSORS: the SDA wires moved to physical 4 and 1).
#   GRAB_TOF - dist1, physical pins 4 (SDA) / 22 (SCL), SoftI2C, at the FRONT, looking
#              through the gripper. Used ONLY at the pick-up: the gripper
#              closes when it reads GRAB_STOP_MM (50 mm).
#   REAR_TOF - dist2, physical pins 1 (SDA) / 15 (SCL), SoftI2C, at the BACK of the robot.
#              The robot REVERSES into a drop branch; this sensor tells
#              whether the slot is empty (slot_occupied(), SLOT_OCCUPIED_MM):
#              empty -> turn round and put the box down; full -> drive
#              straight back to the main line.
# (There is no "front sensor for driving" any more: FRONT_TOF was renamed.)
REAR_TOF = "dist2"             # back of the robot (physical 1 / 15): drop slot empty / full
SLOT_READ_MS = 250             # collect readings this long when checking a slot
SLOT_OCCUPIED_MM = 170         # TODO measure: a box in the slot reads closer than this (rear sensor)
GRAB_TOF = "dist1"             # front, through the gripper (physical 4 / 22): the box at a pick-up
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

# Not used in the final schematic: GP0-GP3 if motor driver 1 is not used.
# (CHANGED 2026-10-23: GP12 is now the push button, see PUSH_BUTTON_PIN.)
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
# ADDED (2026-10-23): safe angle range (degrees, (lowest, highest)) of each
# gripper servo, measured on the robot. Outside it the gripper runs into its
# mechanical end stop: the servo stalls, draws a large current and can pull
# the supply down so far that the Pico resets (this happened on the bench).
# actuators/servo.py clamps every angle into this range. None = not measured
# yet: the sensor viewer / stream then refuse to move that servo at all.
# CHANGED (2026-10-26): the gripper hardware was changed and recalibrated:
# servo1 155 - 173 (first measured as 155 - 187, then the top lowered to 173),
# servo2 130 - 171 (before the change: 170 - 195 and 220 - 270).
# The pick-up (main.py "pick_up") uses the ends of these ranges: servo1 low =
# arm down, servo1 high = arm up, servo2 low = gripper closed on the box.
# Moving a servo outside its range, or the wrong way past an end, breaks the
# gripper. Angles above 270 need pulses longer than the servo's maximum of
# 2.5 ms (at 300 a servo ran away), so 270 is also a hard limit in
# actuators/servo.py, whatever is written here.
SERVO_SAFE_RANGE = {"servo1": (155, 173), "servo2": (132, 170)}
# ADDED (2026-10-23): the angle at which the gripper holds a box (measured
# with the sensor viewer's Grab / Servo tab). servo2 never goes below it.
# CHANGED (2026-10-26): emptied - the old grab angle (servo2 220) belonged to
# the old hardware. TODO measure the new one and put it here.
SERVO_GRAB_ANGLE = {}
# ADDED (2026-10-23): how a servo is RESET, i.e. brought to its start position
# when it has had no pulses (after power-up, after a restart, after going
# limp): (start, end) = go to `start`, wait SERVO_RESET_SETTLE_MS until it is
# there, then turn slowly to `end`, one degree every SERVO_RESET_STEP_MS.
# A servo not listed here is reset by going straight to the middle of its
# safe range.
# CHANGED (2026-10-26): emptied for the new hardware. Every reset must stay
# inside the safe range: going straight to the middle does (a servo that is
# inside its range only moves between its position and the middle), while a
# (start, end) path first jumps to `start` and sweeps the range. The old path
# (servo2 220 -> 250) is outside the new servo2 range anyway.
SERVO_RESET = {}
SERVO_RESET_SETTLE_MS = 600
SERVO_RESET_STEP_MS = 20

# ADDED (2026-10-26): the pick-up and drop-off with the gripper (main.py).
# Pick-up, when the robot has turned into a red / green pick-up box:
#   1. drive on slowly until the GRAB distance sensor reads GRAB_STOP_MM or
#      less (GRAB_APPROACH_MAX_MS is a safety limit: then it stops anyway),
#   2. servo1 down to its low end, 3. servo2 down to its low end (gripper
#      closes on the box), 4. servo1 up to its high end,
#   5. go on following the route.
# Drop-off, at the end of the route: servo2 opens by GRIPPER_RELEASE_DEG, then
# the robot stops if STOP_AFTER_DROP (for testing one pick-up and drop-off).
# While driving the gripper is held at GRIPPER_CARRY: servo1 up, servo2 open
# (the high ends); it goes there right after the safe reset at the start.
GRAB_STOP_MM = 50              # stop in front of the box at this GRAB sensor distance (mm)
# ADDED (2026-10-27, moved from control/pickup.py): in the pick-up "to_box"
# step, stop and fail if the grab sensor gives no real distance (only None)
# for this long in a row (~3 cm of blind driving at GRAB_APPROACH_SPEED).
# Must be longer than TOF_STALE_MS (validate() checks it), else one slow
# reading would already end the pick-up.
GRAB_NO_READING_MAX_MS = 1000
# ADDED (2026-10-27): the lowest motor power (%) that really moves the robot.
# On the robot 30 % did NOT move it (it stood on the cross line until the
# time limit) while 40 % did; validate() refuses lower speeds below.
MIN_MOVING_SPEED = 40
GRAB_APPROACH_SPEED = 50       # CHANGED (2026-10-27, was 30): motor power (%) while driving up to the box
GRAB_REVERSE_SPEED = 70        # Reverse speed
GRAB_STEER = 15                # the wheel on the line's side runs this much slower (%) to steer along it
GRAB_APPROACH_MAX_MS = 4000    # never drive towards the box longer than this
GRIPPER_STEP_MS = 10           # servos turn 1 degree every this many ms (gentle, no jumps)
GRIPPER_PAUSE_MS = 300         # wait after each gripper movement
GRIPPER_CARRY = {"servo1": "high", "servo2": "high"}   # pose while driving ("low" / "high" end)
GRIPPER_SAFE = {"servo1": "low", "servo2": "low"}
GRIPPER_RELEASE_DEG = 10       # TODO tune: how far servo2 opens to let go of the box
STOP_AFTER_DROP = True         # stop after the first drop-off (testing the whole sequence once)
# ADDED (2026-10-27): stop on the main line right after the pick-up (U-turn
# done, the box's magnet pole reported). For now the robot does not turn left /
# right because of the pole; set False to go on with the route.
STOP_AFTER_PICKUP = False

# ADDED (2026-10-27): pushing the box into its slot at the drop-off
# (control/dropoff.py release_and_push). After the gripper has let go:
#   1. back off for DROP_BACK_MS,
#   2. close the gripper (servo2) to SERVO_PUSH_ANGLE,
#   3. drive forward for DROP_PUSH_MS: the closed gripper pushes the box in,
#   4. back out for DROP_EXIT_MS,
#   5. open the gripper again (carry pose), ready for the next pick-up.
# All moves are straight, at DROP_MOVE_SPEED. DROP_PUSH_MS longer than
# DROP_BACK_MS = the box is pushed in by about the difference.
# SERVO_PUSH_ANGLE is BELOW servo2's safe range (130 - 171): the user decided
# (2026-10-27) the empty gripper may close to 120 for the push ONLY; every
# other movement still stops at 130.
DROP_MOVE_SPEED = 50           # CHANGED (2026-10-27, was 30 - too weak to move the robot): motor power (%)
                               # for the small moves at the drop-off
DROP_BACK_MS = 400             # TODO tune: back off this long after letting go
DROP_PUSH_MS = 650             # CHANGED (2026-10-27, was 600; the user: +50 ms): drive forward this long, pushing the box in
DROP_EXIT_MS = 800             # TODO tune: then back out this long
SERVO_PUSH_ANGLE = {"servo2": 120}   # gripper closed this far for the push only (min 130 - 10)
# ADDED (2026-10-27): driving INTO the drop branch, before the steps above
# (control/dropoff.py drive_in_and_drop). Right after the turn from the main
# line into the drop branch the robot
#   1. moves the arm (servo1) to DROP_SERVO1_ANGLE,
#   2. drives in for DROP_DRIVE_MS, following the branch's line with the
#      partner's rule (constants.DRIVE_SPEED straight, TURN_SPEED pivots),
#   3. lets go and pushes the box in (release_and_push, steps above).
# Measured with the sensor viewer's drop-off calibration (Grab / Servo tab),
# record of 2026-10-27 11:51: net 6777 ms forward, servo1 169 deg.
# The calibration drove in many short steps; one continuous drive of the
# same time can go a little further (no spin-up between steps): tune here.
DROP_DRIVE_MS = 7277           # CHANGED (2026-10-27, was 6777 from the calibration; the user: +500 ms)
# CHANGED (2026-10-27, was 169): the user asked for the arm 5 degrees higher
# (174), but servo1's safe range ends at 173, so it is 173 (+4), the top.
DROP_SERVO1_ANGLE = 173        # arm angle before letting go; kept inside servo1's safe range (155 - 173)

# ADDED (2026-10-27): the pick-up sequence (control/pickup.py), from the
# moment the robot has turned from the main line into a red / green box:
#   1. "to_cross_line": follow the box's line (steering with the two centre
#      sensors) until all FOUR line sensors see white at the same time - the
#      white cross line in front of the box (PICKUP_CROSS_CONFIRM reads in a
#      row, so one noisy read does not count);
#   2. "to_box": drive straight on (no steering) at GRAB_APPROACH_SPEED until
#      the GRAB distance sensor (GRAB_TOF, front) reads GRAB_STOP_MM or less;
#   3. "grab": stop; servo1 to its low end (arm down), servo2 to its low end
#      (gripper closed) and kept there, then servo1 to its high end (arm up);
#   4. "u_turn": spin on the spot until the centre sensors have left the line
#      and found it again (at least PICKUP_UTURN_MIN_MS);
#   5. "back_to_main": follow the line back until both outer sensors see the
#      main line (the junction), then stop: "done".
# Each step has a safety time limit (..._MAX_MS); when it runs out the robot
# stops and the sequence ends as "failed" with a message.
# ADDED (2026-10-27): the order in which the boxes are collected (MAP.py
# names). functions.py "collect" takes the first one not collected yet.
PICKUP_ORDER = ("GREEN_1", "GREEN_2", "RED_1", "RED_2")
PICKUP_FOLLOW_SPEED = 60       # CHANGED (2026-10-27, was 40): motor power (%) while following the box's line
                               # (steps 1 and 5); 60 = the partner's DRIVE_SPEED
PICKUP_STEER = 20              # CHANGED (2026-10-27, was 15): the wheel on the line's side runs this much
                               # slower (%) - the slow wheel (60 - 20 = 40) must still turn
PICKUP_CROSS_CONFIRM = 2       # all four sensors white this many reads in a row = the cross line
PICKUP_TO_CROSS_MAX_MS = 6000  # step 1 time limit
PICKUP_UTURN_SPEED = 50        # motor power (%) of both wheels while spinning round
# ADDED (2026-10-27): which way to spin for the U-turn in each box. The two
# boxes on each side line are close together (about 17 cm in the video), and
# spinning TOWARDS the neighbouring box swept the line sensors over that box's
# line: the U-turn stopped there (~70 deg) and the robot drove off along the
# wrong line (found in the simulator). So each box spins AWAY from its
# neighbour: GREEN_1 / RED_2 have the neighbour on their left -> spin right.
PICKUP_UTURN_DIRECTION = {"GREEN_1": "right", "GREEN_2": "left", "RED_1": "left", "RED_2": "right"}
PICKUP_REVERSE_MIN_MS = 1400      # the U-turn never ends earlier than this
PICKUP_UTURN_MAX_MS = 6000     # step 4 time limit
PICKUP_BACK_MAX_MS = 8000      # step 5 time limit

# ---------------------------------------------------------------------------
# Backup line follower (control/line_follower.py). Speeds are % of full PWM.
# Start conservative and raise the speed once it follows reliably.
# ---------------------------------------------------------------------------
START_STOP_SENSOR = "left"     # start / stop button: "left" / "right" crash sensor, or "push" = the
                               # push button on PUSH_BUTTON_PIN (CHANGED 2026-10-23: "push" added)
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
    claim(PUSH_BUTTON_PIN, "push button")     # ADDED (2026-10-23)
    claim(IR_RX_PIN, "IR receiver")
    claim(MAG_PIN, "magnetic sensor")
    for colour, pin in LED_PINS.items():
        claim(pin, colour + " LED")
    for name, pin in (("MOTOR_LEFT_DIR_PIN", MOTOR_LEFT_DIR_PIN), ("MOTOR_LEFT_PWM_PIN", MOTOR_LEFT_PWM_PIN),
                      ("MOTOR_RIGHT_DIR_PIN", MOTOR_RIGHT_DIR_PIN), ("MOTOR_RIGHT_PWM_PIN", MOTOR_RIGHT_PWM_PIN)):
        if not BOARD_PINS.get(pin, "").startswith("motor"):
            problems.append("%s = GP%d is not a motor pin in BOARD_PINS" % (name, pin))
    # CHANGED (2026-10-23): "push" (the push button) is allowed too.
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
    # CHANGED (2026-10-27): FRONT_TOF renamed REAR_TOF (see "Task sensors")
    if REAR_TOF not in [t["name"] for t in TOF_SENSORS]:
        problems.append("REAR_TOF %r is not one of TOF_SENSORS" % (REAR_TOF,))
    if GRAB_TOF not in [t["name"] for t in TOF_SENSORS]:
        problems.append("GRAB_TOF %r is not one of TOF_SENSORS" % (GRAB_TOF,))
    if GRAB_TOF == REAR_TOF:
        problems.append("GRAB_TOF and REAR_TOF must be two different sensors")
    if GRAB_REACH_MM >= GRAB_SEE_MM:
        problems.append("GRAB_REACH_MM must be smaller than GRAB_SEE_MM")
    # ADDED (2026-10-27): the long-range driver code is broken (see libs/VL53L0X.py header)
    if TOF_LONG_RANGE:
        problems.append("TOF_LONG_RANGE must stay False: libs/VL53L0X.py set_Vcsel_pulse_period() is buggy")
    # ADDED (2026-10-27): the push angle may go below the safe range (that is
    # its purpose) but must be a real angle no higher than the range's top
    for name, angle in SERVO_PUSH_ANGLE.items():
        safe = SERVO_SAFE_RANGE.get(name)
        if safe is None or not (0 <= angle <= safe[1]):
            problems.append("SERVO_PUSH_ANGLE[%r] must be between 0 and the top of its safe range" % name)
    # ADDED (2026-10-27): the push may go at most 10 degrees below servo2's
    # safe range. control/dropoff.py PUSH_MAX_BELOW_SAFE_DEG = 10 is the
    # guard while the robot runs (it raises a lower angle to that limit);
    # this check reports such a typo before the robot is even started.
    push2 = SERVO_PUSH_ANGLE.get("servo2")
    safe2 = SERVO_SAFE_RANGE.get("servo2")
    if push2 is not None and safe2 is not None and push2 < safe2[0] - 10:
        problems.append("SERVO_PUSH_ANGLE['servo2'] must be at least SERVO_SAFE_RANGE['servo2'][0] - 10"
                        " (control/dropoff.py PUSH_MAX_BELOW_SAFE_DEG)")
    # ADDED (2026-10-27): the pick-up's "no reading" limit must outlast one
    # stale reading, see GRAB_NO_READING_MAX_MS
    if GRAB_NO_READING_MAX_MS <= TOF_STALE_MS:
        problems.append("GRAB_NO_READING_MAX_MS must be longer than TOF_STALE_MS")
    for name in ("DROP_BACK_MS", "DROP_PUSH_MS", "DROP_EXIT_MS", "DROP_DRIVE_MS"):
        if globals()[name] < 0:
            problems.append("%s must not be negative" % name)
    # ADDED (2026-10-27): a typo like 67770 would drive blind for over a minute
    if DROP_DRIVE_MS > 15000:
        problems.append("DROP_DRIVE_MS must be at most 15000 (15 s of driving blind into the branch)")
    # ADDED (2026-10-27): the arm angle before letting go must lie inside
    # servo1's safe range (Servo clamps it anyway; this reports the typo)
    safe1 = SERVO_SAFE_RANGE.get("servo1")
    if safe1 is not None and not (safe1[0] <= DROP_SERVO1_ANGLE <= safe1[1]):
        problems.append("DROP_SERVO1_ANGLE must lie inside SERVO_SAFE_RANGE['servo1'] %s" % (safe1,))
    if not 1 <= DROP_MOVE_SPEED <= 100:
        problems.append("DROP_MOVE_SPEED must be 1 - 100 (0 would make Robot.forward use full speed)")
    # ADDED (2026-10-27): every speed the pick-up / drop-off drive with must
    # really move the robot (30 % did not on the table)
    for name, speed in (("GRAB_APPROACH_SPEED", GRAB_APPROACH_SPEED),
                        ("PICKUP_FOLLOW_SPEED", PICKUP_FOLLOW_SPEED),
                        ("PICKUP_FOLLOW_SPEED - PICKUP_STEER (the slower wheel)", PICKUP_FOLLOW_SPEED - PICKUP_STEER),
                        ("PICKUP_UTURN_SPEED", PICKUP_UTURN_SPEED),
                        ("DROP_MOVE_SPEED", DROP_MOVE_SPEED)):
        if speed < MIN_MOVING_SPEED:
            problems.append("%s is %d %%, below MIN_MOVING_SPEED (%d %%): the robot would not move"
                            % (name, speed, MIN_MOVING_SPEED))
    # ADDED (2026-10-27): U-turn directions are "left" or "right"
    for box, direction in PICKUP_UTURN_DIRECTION.items():
        if direction not in ("left", "right"):
            problems.append("PICKUP_UTURN_DIRECTION[%r] must be 'left' or 'right'" % box)
    # ADDED (2026-10-26): steering must leave the slower wheel turning (a
    # speed of 0 makes Robot.forward() use the full DRIVE_SPEED instead)
    if not 0 <= GRAB_STEER < GRAB_APPROACH_SPEED:
        problems.append("GRAB_STEER must be at least 0 and smaller than GRAB_APPROACH_SPEED")
    if MAG_NORTH_SIGN not in (1, -1):
        problems.append("MAG_NORTH_SIGN must be 1 or -1")
    for pole in ("north", "south"):
        if MAG_POLE_LEVEL.get(pole) not in ("TOP", "BOTTOM"):
            problems.append("MAG_POLE_LEVEL[%r] must be 'TOP' or 'BOTTOM'" % pole)
    if MAG_NO_POLE_LEVEL not in ("TOP", "BOTTOM"):
        problems.append("MAG_NO_POLE_LEVEL must be 'TOP' or 'BOTTOM'")
    # ADDED (2026-10-23): every servo's safe range is (low, high) inside
    # 0 .. 270 (270 = 2.5 ms pulse, the servo's maximum; beyond it the
    # servo runs away)
    for name in SERVO_PINS:
        safe = SERVO_SAFE_RANGE.get(name)
        if safe is not None and not (0 <= safe[0] < safe[1] <= 270):
            problems.append("SERVO_SAFE_RANGE[%r] must be (low, high) with 0 <= low < high <= 270" % name)
        # ADDED (2026-10-23): a reset path must stay inside the safe range
        path = SERVO_RESET.get(name)
        if path is not None and (safe is None or not (safe[0] <= path[0] <= safe[1] and safe[0] <= path[1] <= safe[1])):
            problems.append("SERVO_RESET[%r] must start and end inside SERVO_SAFE_RANGE[%r]" % (name, name))
        # ADDED (2026-10-26): the gripper's carry pose names an end of the range
        if GRIPPER_CARRY.get(name, "high") not in ("low", "high"):
            problems.append("GRIPPER_CARRY[%r] must be 'low' or 'high'" % name)
        # ADDED (2026-10-23): the grab angle must be inside the safe range too
        grab = SERVO_GRAB_ANGLE.get(name)
        if grab is not None and (safe is None or not (safe[0] <= grab <= safe[1])):
            problems.append("SERVO_GRAB_ANGLE[%r] must be inside SERVO_SAFE_RANGE[%r]" % (name, name))
    return problems


def sensor_pins():
    """{GP: description} for every pin the sensors use."""
    pins = {
        LINE_OUTER_LEFT_PIN: "line outer-left", LINE_INNER_LEFT_PIN: "line inner-left",
        LINE_INNER_RIGHT_PIN: "line inner-right", LINE_OUTER_RIGHT_PIN: "line outer-right",
        CRASH_LEFT_PIN: "crash left", CRASH_RIGHT_PIN: "crash right",
        IR_RX_PIN: "IR receiver", MAG_PIN: "magnetic sensor",
        PUSH_BUTTON_PIN: "push button",     # ADDED (2026-10-23)
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
