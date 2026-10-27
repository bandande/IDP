# IDP - Robot Code and Bench Guide

Cambridge Part IB Integrated Design Project. A line-following robot on a
Raspberry Pi Pico 2 (RP2350) running MicroPython. This README explains what
is in the repository and how the main program works (section 3), how every
sensor is wired, how to get the code onto the Pico, a step-by-step bench
test plan with pass criteria, calibration sheet and troubleshooting, and the
PC simulator for testing the program without the robot.

All pins follow the **final team schematic** `SCH_Schematic1_2026-10-15.pdf`
(EasyEDA, page "sensor_schematic_1"). `config.py` is the single place where
pins are defined. **The schematic has one drawing error that must not be
copied into the wiring: see section 5, "Schematic error".**

---

## 1. Status (2026-10-20)

Main program: the team's `main.py` / `functions.py` / `MAP.py` (queue of
tasks, see section 3), with the fixes of 2026-10-20 (each marked FIX /
ADDED / CHANGED in the code) and the task sensors (magnet pole, slot check,
traffic light). It passes all simulator checks (`python -m sim.check`,
6 of 6) up to the first box drop, but the fixes have **not run on the real
table yet**; distances marked TODO in `main.py` and `config.py` still have
to be measured.

| Sensor | Count | Code | Tested on the robot |
|---|---|---|---|
| Line tracking, DFRobot SEN0017 | 4 | `sensors/line_sensor.py`, `sensors/line_processing.py` | 2 sensors worked on temporary pins; re-test on the final pins |
| Distance (ToF), VL53L0X | 2 | `sensors/tof.py` | one sensor worked on GP8/GP9 earlier (6 min, 0 I2C errors, about 35 readings/s, 24-829 mm); re-test as dist1 (grab, front) / dist2 (rear) on the pins of 2026-10-27 |
| Crash, DFRobot SEN0138 L/R | 2 | `sensors/crash.py` | yes, on physical 11 / 12; re-check which lever is left (final schematic: L = 12, R = 11) |
| IR receiver, Vishay TSOP4838 | 1 | `sensors/ir_receiver.py` | yes, on physical 19: 1 Hz IR beacon (500 ms on / 500 ms off) received, no false frames |
| Magnetic (Hall, analog output) | 1 | `sensors/magnetic.py` | yes, on GP26; analog, thresholds set |

The board: `Raspberry Pi Pico2 with RP2350`, MicroPython v1.28.0, appears as a
USB serial port (COM8 on the laptop used so far, USB ID 2E8A:0005).

---

## 2. Bring / install before the session

Hardware
- Robot board with the Pico 2, USB cable
- Multimeter
- A strip of white tape on black card (line sensors)
- A flat light-coloured target and a ruler (ToF calibration)
- A permanent magnet (Hall sensor)
- Any TV remote control (IR receiver test; most use the NEC code)

Software on the laptop
- VS Code with the MicroPico extension (`paulober.pico-w-go`). Opening this
  folder offers it automatically (`.vscode/extensions.json`).
- Python 3 with pyserial for the live viewer: `pip install pyserial`
- Optional: `pip install mpy-cross` to check that a file compiles for MicroPython (section 13)

---

## 3. Repository layout

| Path | Runs on | Contents |
|---|---|---|
| `main.py` | Pico | the main program (the team's): a queue of tasks (collect, deliver, go back) worked through by a mode state machine; turn / junction settings at the top. The Pico runs it automatically at every boot. |
| `functions.py` | Pico | `Robot`: position, heading, the task queue, motor moves (`forward`, `spin`, ...), `find_route()`, `move_to()` |
| `MAP.py` | Pico | the table as nodes with their north / east / south / west neighbours |
| `motor.py`, `constants.py` | Pico | the team's `Motor` class (DIR + PWM) and constants |
| `config.py` | Pico | every pin and sensor setting, including the task sensors; `validate()` (errors), `warnings()` (pins to check), `physical_pin(gp)` |
| `sensors/` | Pico | one module per sensor: hardware interface + data processing |
| `actuators/` | Pico | `motor.py` (drive motors, `DriveBase`), `servo.py` (gripper servos), `gripper.py` (gripper moves: start, grab, release), `leds.py` (yellow / red / green LEDs) |
| `control/` | Pico | `task_sensors.py` (magnet pole, slot check, traffic light), `pickup.py` (pick-up sequence), `dropoff.py` (let go + push the box in), `start_stop.py` (crash sensor as start / stop button), `line_follower.py` (backup line follower for the bench test) |
| `sim/` | PC | the simulator: runs the robot code unchanged on a simulated table (see "Simulator" below). Do not upload. |
| `libs/VL53L0X.py` | Pico | VL53L0X driver from the department template (unchanged) |
| `tests/test_*.py` | Pico | on-robot test scripts, one per sensor, numbered in the recommended order |
| `tests/stream_sensors.py` | Pico | streams every reading to the PC as JSON lines for the viewer |
| `tools/sensor_viewer.py` | PC | live viewer window. Not uploaded to the Pico (excluded in `.vscode/settings.json`). |
| `.micropico`, `.vscode/` | PC | MicroPico project settings |

Sensor modules

| Module | Main API |
|---|---|
| `sensors/line_sensor.py` | `LineSensorArray.from_config()`, `.read()` -> 4-bit mask (bit set = white), `.read_raw()` |
| `sensors/line_processing.py` | bit masks `OUTER_LEFT` ... `OUTER_RIGHT`; `LineTracker` (steering error -1..+1, ON_LINE / SEARCHING / LOST), `JunctionDetector` (ENTER / EXIT events with shape FOUR_JUNCTION / T_JUNCTION / CORNER; used by the backup follower), `bits_to_str()` |
| `sensors/tof.py` | `tof_from_config()` -> `{"dist1": ToFSensor, "dist2": ToFSensor}`; `.poll()`, `.distance_mm` (None = no target / stale / unavailable), `.raw_mm`, `.range_status`, `.is_closer_than(mm)`, `.begin()` |
| `sensors/crash.py` | `BumperPair.from_config()`, `.update()`, `.which()` -> None / "LEFT" / "RIGHT" / "BOTH", `.consume_hits()` |
| `sensors/ir_receiver.py` | `IRReceiver.from_config()`, `.process()`, `.signal_present()`, `.get_frame()`; `decode_nec(frame)`, `match_template(frame, template)`, `frame_signature(frame)` |
| `sensors/magnetic.py` | `HallSensor.from_config()`, `.calibrate_baseline()`, `.update()`, `.detected`, `.consume_detection()`, `.field_mv`, `.polarity` |
| `sensors/filters.py` | `MedianFilter`, `Debouncer`, `Hysteresis` |
| `actuators/motor.py` | `DriveBase.from_config()`, `.drive(left, right)` (percent, -100 .. +100), `.stop()` |
| `actuators/leds.py` | `StatusLeds.from_config()`, `.set("green", True)`, `.show(red=True)`, `.off()` |
| `control/line_follower.py` | `LineFollower.from_config()`, `.start(plan=None)`, `.update(bits)` -> (left, right), `.stop()`, `.state`, `.reason` |
| `control/start_stop.py` | `StartStopButton.from_config()`, `.pressed()` (True once per press) |
| `control/task_sensors.py` | `TaskSensors.from_config()`, `.calibrate()`, `.read_pole()` -> "north" / "south" / None, `.level_for_pole()`, `.slot_occupied()` / `.rear_distance_now()` (rear sensor), `.box_distance_now()` / `.box_distance()` -> mm or None, `.box_seen(d)`, `.box_in_reach(d)` (grab sensor), `.light_reset()`, `.light_update()` -> "red" / "green"; the two distance sensors themselves are `.rear_tof` and `.grab_tof` |
| `actuators/servo.py` | `Servo(pin)` (safe range from `config.SERVO_SAFE_RANGE`), `.reset()` (required first move, waits until there), `.set_angle(deg)` / `.move_slowly(deg)` (clamped into the safe range), `.angle()` (last angle set), `.off()` |
| `control/pickup.py` | `PickUp(robot, motor3, motor4, servo1, servo2, tasks, uturn_direction)`, `.start(now_ms)`, `.update(now_ms)` -> True when finished, `.ok`, `.phase`, `.message`, `.pole` / `.level` (box magnet) |
| `control/dropoff.py` | `release_and_push(robot, motor3, motor4, servo1, servo2)` -> True when done |

### The main program (`main.py`)

**CHANGED (2026-10-26): `main.py`, `functions.py` and `constants.py` are now
the partner's version from `main` (commit b4ff1d8: line following and
navigation in `functions.robot_mode_check()`, all state in one tuple), so that
the robot is tested with the partner's line following. Our gripper sequence
is added to its `pick_up` / `drop_off` (`functions.gripper_start`,
`grab_box`, `release_box`; marked "test build"), and its servo1 / servo2 are
now taken by name (MicroPython dicts do not keep their order). Our own
navigation (CLOSE_AFTER_TURN, check_slot, ... described below) is in the
history of this branch (commit 3598b7c). In the simulator this version passes
1 of 6 checks: the partner's turning values were tuned on the real robot and
it does not turn at MIDDLE_LEFT in the sim; the result is the same without
the gripper changes.**

The robot works through a queue of tasks (`robot.queue`, used from the end):
`collect` (plan the route to the next pad + `pick_up`), `deliver` (plan the
route to the first free drop slot + `drop_off`) and `go_back` (route to START
+ `stop`). Node names are expanded by `robot.move_to()` into a turn and a
"straight" drive; the modes (`straight`, `correction`, `turning`, ...) carry
them out. Changes of 2026-10-20, all marked in the code:

- **Start / stop button** (left front crash sensor): read once per loop
  (before, a press was often used up and lost, so the button sometimes did
  not stop the robot). A SHORT press pauses and the next press continues
  from the same place; a LONG press (`LONG_PRESS_MS`) stops and resets to
  START.
- **Task sensors** (`control/task_sensors.py`, settings in `config.py`):
  - at `pick_up` the magnet pole decides the level (`drop_off_info[1]`,
    TOP / BOTTOM drops);
  - after turning into a drop slot, `check_slot` looks into it with a
    distance sensor (CHANGED 2026-10-27: now the REAR sensor, the robot
    reverses into the slot); a full slot is marked `OCCUPIED` and the next
    one is planned (all full -> reset, as before);
  - before driving up the ramp (START -> TOP_CENTRE) the robot waits for the
    traffic light (`wait_light`, IR beacon = green, no time limit).
- **Gripper** (2026-10-26, settings in `config.py` under "Gripper servos"):
  - after the start press both servos are reset the safe way (straight to
    the middle of their safe range) and go slowly to the carry pose (servo1
    up, servo2 open);
  - `pick_up` (CHANGED 2026-10-27: `control/pickup.py`): after turning into
    the red / green pick-up box the robot follows its line to the white cross
    line, drives on straight until the GRAB distance sensor reads
    `GRAB_STOP_MM` (50 mm), then servo1 down, servo2 closed, servo1 up,
    reads the box magnet, turns round and follows the line back to the main
    line (see Step 11);
  - `drop_off` (CHANGED 2026-10-27: `control/dropoff.py`): servo2 opens by
    `GRIPPER_RELEASE_DEG`, the robot backs off, closes the gripper to
    `SERVO_PUSH_ANGLE`, pushes the box in and backs out; with
    `STOP_AFTER_DROP` the robot then stops (a button press goes on);
  - servo safe ranges (new gripper): servo1 155-173, servo2 130-171 deg. No
    angle outside them is ever sent, every movement is one degree at a time,
    and a servo with no pulses only starts with its safe reset.
- **Nodes closer than the line sensors** (`CLOSE_AFTER_TURN`,
  `close_after_turn()`): the sensors are ~20 cm in front of the wheels, so
  after a turn on the spot a node only 8-17 cm away (start box edges, the
  second pad junction, the next drop junction) is already behind them. It
  is counted at once, and the next turn only drives the remaining distance.
- **Junction hold-off** only after a junction is detected, not after a turn.
- **`TURN_MIN_MS`** is no longer reassigned inside `main()` (local
  `turn_min`, `START_TURN_MIN_MS` for the first turn).
- **`MAP.py`**: wrong links fixed (east line, spine end, T-bars); the
  START.west "map bodge" is replaced by `CLOSE_AFTER_TURN`.
- `LINE_TEST_ONLY` and the line-test route were removed from `main.py`
  (bench line tests: `tests/test_line_follow.py`).

### Simulator (`sim/`, runs on the PC)

The simulator runs the robot code **unchanged** on a simulated table built
from `sim/track_map.py` (the table with positions in metres; the robot
itself uses `MAP.py`): the robot's motion (rear-wheel drive, sensors 20 cm
in front), line sensors, start button, magnets in the pads, boxes in rack
slots (rear distance sensor), the box ahead of the grab sensor and the IR
traffic light. Needs `numpy` and
`matplotlib`.

```
python -m sim.check                     # all regression checks, PASS / FAIL (~1 min)
python -m sim.run --png run.png         # one run of main.py, with a picture of the path
python -m sim.run --boxes LEFT_BOTTOM_DROP_1 --poles GREEN_1=south --light 20,10
python -m sim.arena --png table.png     # draw the table with the node names
```

Run `python -m sim.check` before every push. The robot's size and speed in
the simulator (`sim/world.py`, `RobotSpec`) and the table
(`sim/track_map.py`) are estimates: a PASS means the logic works, not that
the real robot will.

Every module starts with a docstring that explains the device, its wiring, the
data processing and a usage example.

---

## 4. Reading the Pico pinout

- Two numbering schemes. **Code uses GP numbers** (`Pin(18)` is GP18). **The
  jumpers on the robot board and the schematic pin numbers are physical pin
  numbers 1-40.** They are different: GP18 is physical pin 24.
  `config.physical_pin(18)` returns 24, and every pin in `config.py` has both
  numbers in its comment.
- Physical numbering: board seen from the top, USB socket at the top. Pin 1
  is top-left, numbers go down the left side to 20, continue at the bottom
  right with 21 and go up the right side to 40.
- Every fifth pin is a GND: 3, 8, 13, 18, 23, 28, 33 (AGND), 38.
- Power pins: 36 = 3V3(OUT), the 3.3 V supply for all sensors in the
  schematic. 39 = VSYS. 40 = VBUS, 5 V **only while USB is plugged in**. Do not
  connect anything to 37 (3V3_EN), 30 (RUN) or 35 (ADC_VREF).
- Only GP26, GP27, GP28 (physical 31, 32, 34) can read analog voltages.
- I2C: for HARDWARE I2C, SDA and SCL must belong to the same bus (I2C0 or
  I2C1). CHANGED (2026-10-27): after the team re-wired the SDA wires, the
  ToF sensors use software I2C (SoftI2C, any two pins), one sensor per bus:
  dist1 on GP2/GP17 (physical 4/22), dist2 on GP0/GP11 (physical 1/15). The
  schematic still shows the old pins (GP16/GP17 and GP10/GP11).
- **All GPIO pins are 3.3 V. Never put 5 V on a GPIO pin.**

### Header map (final schematic, distance sensors re-wired 2026-10-27)

```
 phys GP    use (left side)                             phys GP    use (right side)
    1 GP0   Distance Sensor 2  D (SDA)  SoftI2C *         21 GP16  free (was Distance Sensor 1 SDA)
    2 GP1   motor driver 1 #1 PWM (not in schematic)      22 GP17  Distance Sensor 1  C (SCL)  SoftI2C
    3 GND                                                 23 GND   optocoupler ground
    4 GP2   Distance Sensor 1  D (SDA)  SoftI2C *         24 GP18  Line Sensor Edge Right    OUT
    5 GP3   motor driver 1 #2 (assumed)                   25 GP19  Line Sensor Centre Right  OUT
    6 GP4   DIR1   motor 1 direction                      26 GP20  Line Sensor Edge Left     OUT
    7 GP5   PWM1   motor 1 speed                          27 GP21  Line Sensor Centre Left   OUT
    8 GND                                                 28 GND
    9 GP6   PWM2   motor 2 speed                          29 GP22  Yellow LED (via optocoupler)
   10 GP7   DIR2   motor 2 direction                      30 RUN   do not use
   11 GP8   Crash Sensor R  DIG/S                         31 GP26  Magnetometer OUT (ADC0)
   12 GP9   Crash Sensor L  DIG/S                         32 GP27  Red LED (via optocoupler)
   13 GND                                                 33 AGND
   14 GP10  free (was Distance Sensor 2 SDA)              34 GP28  Green LED (via optocoupler)
   15 GP11  Distance Sensor 2  C (SCL)  SoftI2C           35 VREF  ADC_VREF, do not use
   16 GP12  Push button DIG/S (optional start button)     36 3V3   3V3_OUT: power for ALL sensors
   17 GP13  ServoPin2  servo 2 signal                     37 3V3EN 3V3_EN, do not use
   18 GND                                                 38 GND   ground for ALL sensors
   19 GP14  IR Sensor OUT                                 39 VSYS
   20 GP15  ServoPin1  servo 1 signal                     40 VBUS  5 V only with USB
```

- Motors and servos are the schematic's net labels DIR1 / PWM1 / DIR2 /
  PWM2 / ServoPin1 / ServoPin2 (the same pins as the department examples).
- The three indicator LEDs are switched through a 4-channel optocoupler:
  GPIO HIGH -> LED on (`config.LED_PINS`).
- \* GP0 / GP2 may also be routed to motor driver 1 on the department board:
  nothing may be plugged into motor driver 1 (`config.BOARD_CHECKS`).
- Free pins (CHANGED 2026-10-27): GP10 and GP16 (physical 14 and 21), plus
  GP1 / GP3 if motor driver 1 is not used.

---

## 5. Wiring reference (final schematic)

Do all wiring with the USB cable unplugged. Every sensor gets VCC from
**3V3_OUT (physical 36)** and GND from **GND (physical 38)**; any other GND pin
works as well. No sensor needs extra parts.

| Sensor (schematic name) | Sensor pins | Signal -> Pico (physical / GP) | Code name |
|---|---|---|---|
| Line Sensor Edge Left | 1 OUT, 2 VCC, 3 GND | OUT -> 26 / GP20 | `LINE_OUTER_LEFT_PIN` |
| Line Sensor Centre Left | 1 OUT, 2 VCC, 3 GND | OUT -> 27 / GP21 | `LINE_INNER_LEFT_PIN` |
| Line Sensor Centre Right | 1 OUT, 2 VCC, 3 GND | OUT -> 25 / GP19 | `LINE_INNER_RIGHT_PIN` |
| Line Sensor Edge Right | 1 OUT, 2 VCC, 3 GND | OUT -> 24 / GP18 | `LINE_OUTER_RIGHT_PIN` |
| Distance Sensor 1 (`+ - C D`) | 1 VCC, 2 GND, 3 SCL (C), 4 SDA (D) | D -> 21 / GP16, C -> 22 / GP17 (I2C0) | `"dist1"` |
| Distance Sensor 2 (`+ - C D`) | 1 VCC, 2 GND, 3 SCL (C), 4 SDA (D) | D -> 14 / GP10, C -> 15 / GP11 (I2C1) | `"dist2"` |
| Crash Sensor L | 1 DIG/S, 2 VCC, 3 GND | DIG/S -> 12 / GP9 | `CRASH_LEFT_PIN` |
| Crash Sensor R | 1 DIG/S, 2 VCC, 3 GND | DIG/S -> 11 / GP8 | `CRASH_RIGHT_PIN` |
| IR Sensor TSOP4838 | 1 OUT, 2 GND, 3 VCC (VS) | OUT -> 19 / GP14 | `IR_RX_PIN` |
| Magnetometer (Hall, analog) | 1 OUT, 2 GND, 3 VS | OUT -> 31 / GP26 (ADC0) | `MAG_PIN` |

Other connections in the schematic: motors DIR1 = GP4, PWM1 = GP5, PWM2 = GP6,
DIR2 = GP7; servos ServoPin1 = GP15, ServoPin2 = GP13; LEDs yellow = GP22,
red = GP27, green = GP28 (through the optocoupler, 220 ohm resistors).

**Schematic error.** The schematic draws Distance Sensor 1's SDA (pin 4) and
VCC (pin 1) joined to the same vertical wire, which also runs to physical 21
(GP16) and the 3.3 V rail. Wired like that, SDA and GP16 would be shorted to
3.3 V: I2C cannot work and the Pico pin can be damaged when it drives SDA
low. Wire **SDA only to physical 21** and **VCC only to physical 36**, and
before power-up check with a multimeter that physical 21 and 36 are NOT
connected. `config.warnings()` and `tests/test_wiring.py` print this reminder.

Notes
- Line sensors, left to right across the robot: Edge Left, Centre Left,
  Centre Right, Edge Right. In the code the Edge sensors are "outer" and the
  Centre sensors "inner".
- The ToF boards have only 4 wires (no XSHUT), so both keep address 0x29.
  That is why each one has its own I2C bus.
- Crash sensors: the schematic powers them from 3.3 V, so their output is at
  most 3.3 V and goes straight to the Pico. DFRobot rates the board for 5 V;
  the micro-switch and its pull-up work at 3.3 V, only the LED is dimmer.
  **If they are ever moved to 5 V, a divider (10k series, 20k to GND) between
  DIG/S and the Pico pin becomes mandatory.**
- IR receiver: lens facing you, legs down, left to right: 1 OUT, 2 GND, 3 VS.
  Optional supply filter if it is noisy near the motors: about 100 ohm in
  series with VS and a few uF from VS to GND, close to the sensor.
- Follow the letters printed on each board, not the wire colours.
- Total 3.3 V load (4 line + 2 ToF + 2 crash + IR + Hall) is about 130 mA,
  within what 3V3(OUT) can supply.

---

## 6. Getting the code onto the Pico

1. Open this folder in VS Code. MicroPico connects to the Pico automatically.
2. Command palette -> "MicroPico: Upload project to Pico". This copies
   `config.py`, `sensors/`, `libs/`, `tests/`, `main.py` etc. `tools/` is skipped.
3. To run a test: open the file (e.g. `tests/test_wiring.py`) and use
   "Run current file on Pico". Stop it with Ctrl-C in the MicroPico terminal.
   Alternative in the REPL: `from tests.test_wiring import test_wiring; test_wiring()`.
4. After uploading new files, soft-reset (Ctrl-D in the REPL) so the Pico
   does not keep using an older module it has already imported.

Important
- **Only one program can use the serial port at a time.** Disconnect
  MicroPico (status bar) before starting the live viewer, and close the
  viewer before using MicroPico again.
- The Pico runs `main.py` at every boot and after every soft reset. It does
  not move until the start button is pressed, but it keeps the Pico busy:
  press Ctrl-C to stop it before running a test (the motors are switched
  off when it stops).

---

## 7. Bench test procedure

Go in this order and only wire the sensors needed for each step. Each step
says what "pass" looks like. Write the results into the sheet in section 9.

### Step 0 - before power-up
- [ ] Every sensor VCC goes to physical 36 (3V3_OUT), every GND to a GND pin.
- [ ] Nothing is connected to 37, 39, 40 by mistake (37 is next to 36).
- [ ] With the power off: physical 21 and 36 are NOT connected (schematic
      error, section 5).

### Step 1 - wiring check: `tests/test_wiring.py`
Prints the config check and the pins to check on the board, scans both ToF
I2C buses and prints the raw level of every sensor pin once per second.
- Pass: `config.validate(): OK`; the "check on the board" line (Distance
  Sensor 1 SDA) has been checked; each connected ToF bus shows `[41]` (0x29);
  pin levels change when you act on a sensor.

### Step 2 - line sensors: `tests/test_line_sensors.py`
Mount the sensors 1-2 cm above the floor, in a row across the robot.
1. Over black floor the pattern is `....`.
2. Put white tape under each sensor in turn: the matching character becomes
   `#`, in the order Edge Left, Centre Left, Centre Right, Edge Right. Wrong
   position -> the wires do not match the schematic, or fix `config.py`.
   Everything inverted -> set `LINE_WHITE_IS_HIGH = False`.
3. Robot centred on the line: `.##.` and error `+0.00`. Slide it sideways:
   line to the left of the robot gives a negative error.
4. Push the robot slowly over a junction: `ENTER` then `EXIT` events.
- Pass: all four sensors respond and are in the right order.
- Geometry assumption used by the code: the two Centre sensors are closer
  together than the line width (both on white when centred); each Edge
  sensor is more than one line width further out. Measure the line width and
  note the spacing you used.
- The outputs are digital (0/1 only); the SEN0017 has no analog output.

### Step 3 - ToF distance sensors: `tests/test_tof.py` (or the viewer, Distance tab)
Peel the protective film off the sensor windows.
1. Both sensors show `available True` (else check the wiring, section 11).
2. Move a hand or box between 100 and 500 mm: the distance follows, status 11.
3. Nothing in front: distance `None` (raw 8190/8191, status 4).
4. Calibrate the offset: flat target exactly 100 mm from the window, then in
   the REPL `from tests.test_tof import calibrate_offset; calibrate_offset("dist1", 100)`
   (or the Calibrate button in the viewer). Put the printed `offset_mm` into
   `config.TOF_SENSORS`. Repeat for "dist2".
5. Wiggle the cables: the I2C error count must stay at 0. Rising errors mean
   a loose wire; a sensor that lost power stops measuring until it is
   re-initialised (press Apply in the viewer or restart the test).
- Pass: steady readings (noise a few mm), 0 I2C errors, offsets recorded.
- Mounting: the field of view is about 25 degrees, so the floor enters the
  cone at about 4.5 x the mounting height. Mount at least about 5 cm high or
  tilt slightly upwards, then check that the floor is not detected.

### Step 4 - IR receiver: `tests/test_ir.py`
1. At rest: `IR absent`, no frames.
2. Point a TV remote at it and press a button: `IR PRESENT`, frames are
   printed; an NEC remote shows an address and command.
3. If the course traffic light is available: point it at the receiver and
   copy the printed `template:` list. Later `match_template(frame, TEMPLATE)`
   recognises it.
- Pass: a remote is decoded or at least produces frames.
- Note: the TSOP4838 suppresses a continuous 38 kHz signal. A beacon that is
  simply "always on" may only give one short pulse.

### Step 5 - magnetic sensor: `tests/test_magnetic.py` (or the viewer, Magnetic tab)
The script first measures the zero-field baseline: keep magnets away.
1. No magnet: field within about +/-100 mV, shows `-`.
2. Magnet within 1 cm of the chip face: `MAGNET`, field several hundred mV.
   The other pole moves the field the other way.
3. Take it away: back to `-` within a second.
- Pass: every approach is detected, no false detections at rest.
- Measured so far: zero field about 2.1-2.25 V, drifts by about 100 mV, a
  close magnet saturates at about 0.05 V / 3.0 V. Thresholds in `config.py`:
  `MAG_ON_MV = 300`, `MAG_OFF_MV = 200`. Note the distance at which a box
  magnet is still detected.

### Step 6 - crash sensors: `tests/test_crash.py`
1. Check that VCC goes to 3V3_OUT (36), not 5 V. Measure DIG/S with a
   multimeter: about 3.3 V released, about 0 V pressed.
2. Signals: left on physical 12 (GP9), right on physical 11 (GP8).
3. At rest: raw 1, pressed False. Press a lever: raw 0, pressed True, one hit
   per press. A quick tap still counts exactly once.
- Pass: both sensors, correct side (L = GP9, R = GP8), one hit per press.
- An unplugged sensor may read "released" on this chip (RP2350 erratum E9),
  so always do the press test after wiring.

### Step 7 - everything together: `tests/test_all_sensors.py` (or the viewer, Overview tab)
Run with all sensors connected, then again with the drive motors running.
- Pass: no I2C errors, no false crash hits or IR frames while the motors run.

### Step 8 - start / stop button and LEDs: `tests/test_start_button.py`
The button is the crash sensor named in `config.START_STOP_SENSOR` (default
"left" = physical 12). Nothing moves.
1. At start each LED lights for half a second: yellow, red, green.
2. Red = STOP. One press: green = RUN (yellow flashes on every press). Press
   again: back to STOP.
- Pass: all three LEDs work; exactly one toggle per press.

### Step 9 - drive motors: `tests/test_motors.py`
**First run with the wheels off the table.** Nothing moves until the button
is pressed; another press (or Ctrl-C) stops everything.
1. `test_motors()`: left wheel forward, right wheel forward, both forward,
   spin left, spin right (1 s each, 40 %).
   - Wrong wheel in step 1 -> swap the `MOTOR_LEFT_*` and `MOTOR_RIGHT_*` pins.
   - A wheel turns backwards -> flip `MOTOR_LEFT_INVERT` / `MOTOR_RIGHT_INVERT`.
2. `ramp_test()`: both wheels from 5 % to 60 %. The first speed at which both
   wheels turn goes into `MOTOR_MIN_DUTY`.
- Pass: the five steps move exactly as described.

### Step 10 - backup line follower: `tests/test_line_follow.py`
1. Dry run first: `test_line_follow(dry_run=True)`, motors stay off. Push the
   robot along the line by hand and watch the printed speeds: line to the
   left -> left speed lower than right.
2. Real run: robot centred on a straight line. Press the button; the yellow
   LED blinks for `START_DELAY_MS`, then it follows. Press again (or bump the
   front) to stop. Ctrl-C always stops the motors.
3. Then bends, a crossing (goes straight), a corner (turns by itself) and a
   plan, e.g. `test_line_follow(plan=["L", "S", "R"])`.
- LEDs: green = following, yellow = turning / on a junction, red = stopped.
  When it stops by itself the reason is printed (line lost, T junction ...).
- Pass: follows a straight line and gentle bends at `FOLLOW_BASE_SPEED`
  without losing it; turns at corners; stops on the button.

How the backup follower works (`control/line_follower.py`)
- Steering: `error` from the line sensors (-1 = line far left .. +1 = far
  right), `correction = FOLLOW_KP * error + FOLLOW_KD * d(error)/dt`,
  limited to `FOLLOW_MAX_CORRECTION`; left wheel = base + correction, right
  wheel = base - correction.
- Line lost for `LINE_LOST_MS`: spin towards the side it was last seen at
  `FOLLOW_SEARCH_SPEED`; stop after `FOLLOW_SEARCH_TIMEOUT_MS`.
- Junctions: without a plan it goes straight over crossings and side
  branches, turns by itself at corners, and does `T_STEM_ACTION` when the
  line ends in a T. A plan gives one action per junction, corners included:
  "L", "R", "S" or "stop".
- Turns: drive on for `TURN_FORWARD_MS` after the junction was first seen
  (so the wheels are over it), then spin at `TURN_SPEED` until a centre
  sensor finds the new line; give up after `TURN_TIMEOUT_MS`.

Tuning (`config.py`)
| Symptom | Change |
|---|---|
| Wobbles from side to side on a straight line | lower `FOLLOW_KP`, or add `FOLLOW_KD` (try 2-5) |
| Leaves the line in bends | raise `FOLLOW_KP` or lower `FOLLOW_BASE_SPEED` |
| Turns too early / too late at a junction | raise / lower `TURN_FORWARD_MS` |
| Overshoots the new line when turning | lower `TURN_SPEED` |
| Wheels hum but do not move at low speed | set `MOTOR_MIN_DUTY` from `ramp_test()` |

---

## 8. Live viewer (`tools/sensor_viewer.py`)

```
pip install pyserial
python tools/sensor_viewer.py                       # pick the port, press Connect
python tools/sensor_viewer.py --tab tof --connect   # open on the Distance tab
python tools/sensor_viewer.py --tab mag --connect --record log.txt
python tools/sensor_viewer.py --tab servo --connect               # gripper servos
python tools/sensor_viewer.py --tab servo --connect --servo-free  # recalibrate after a hardware change
```

- Close MicroPico's connection first. The viewer stops whatever runs on the
  Pico (no soft reset any more: that ran `main.py`) and starts
  `tests/stream_sensors.py`, so the code must be uploaded first.
- Grab / Servo tab: Left / Right move the selected servo inside its safe
  range (Shift = 5 steps, 1 / 2 select, x limp, r safe reset, Enter locks the
  grab angle), next to the distance and magnet readings. `--servo-free`
  ignores the safe ranges in `config.py` (only the 270 degree limit stays):
  for measuring new ranges after the gripper hardware changed.
- Wiring row: type GP numbers (not physical numbers) for each sensor group
  and press Apply. The defaults are the schematic pins from `config.py`.
  Leave a field empty to switch that group off. ToF pins always come from
  `config.py`.
- Overview tab: pin monitor (every header pin except the motor driver pins;
  unused pins read a steady HIGH because their pull-up is on, a connected
  sensor makes its pin change), line pattern and error, distances, crash, IR.
- Distance tab: one card per ToF sensor with the I2C scan result, status,
  rate and noise; continuous distance curve (solid = filtered, dashed = raw);
  offset calibration helper.
- Magnetic tab: continuous voltage curve with baseline and thresholds,
  detection lamp, and a guess of whether the sensor is digital or analog.
- `--record FILE` saves everything the Pico sends, with timestamps, for later analysis.
- Options: `--port`, `--line`, `--crash`, `--ir`, `--mag`, `--no-tof`, `--tab overview|tof|mag`, `--connect`, `--record`.

---

## 9. Calibration and results sheet

Fill this in during the session, then update `config.py`.

| Item | Value | Goes into |
|---|---|---|
| Line sensor order correct (Edge L, Centre L, Centre R, Edge R) | yes / no | `LINE_*_PIN` |
| Line sensors white = HIGH | yes / no | `LINE_WHITE_IS_HIGH` |
| Line width / Centre spacing / Edge spacing (mm) | | (geometry check) |
| Line sensor height above floor (mm) | | |
| dist1 offset at 100 mm (mm) | | `TOF_SENSORS[0]["offset_mm"]` |
| dist2 offset at 100 mm (mm) | | `TOF_SENSORS[1]["offset_mm"]` |
| Where dist1 / dist2 are mounted (front / side ...) | | (rename in `config.py` if wanted) |
| ToF floor visible at mounting height? | yes / no | (mounting) |
| Crash DIG/S voltage released / pressed, left (V) | / | (expect 3.3 / 0) |
| Crash DIG/S voltage released / pressed, right (V) | / | (expect 3.3 / 0) |
| Crash pressed = LOW | yes / no | `CRASH_ACTIVE_LOW` |
| Physical 21 and 36 not connected (continuity test) | yes / no | (section 5) |
| IR: remote decoded (address, command) | | |
| IR: traffic light template frame | | (code) |
| Magnetic baseline (mV) | | (check) |
| Magnetic field with box magnet at working distance (mV) | | `MAG_ON_MV`, `MAG_OFF_MV` |

---

## 10. Using the sensors in the main loop

All `update()` / `poll()` / `read()` calls are non-blocking, so one loop can
service every sensor. Create everything once, then poll every iteration:

```python
from utime import ticks_ms, sleep_ms
from sensors.line_sensor import LineSensorArray
from sensors.line_processing import LineTracker, JunctionDetector
from sensors.tof import tof_from_config
from sensors.crash import BumperPair
from sensors.ir_receiver import IRReceiver, decode_nec
from sensors.magnetic import HallSensor

line = LineSensorArray.from_config()
tracker = LineTracker.from_config()
junctions = JunctionDetector.from_config()
tofs = tof_from_config()            # blocking start-up, a fraction of a second
bumpers = BumperPair.from_config()
ir = IRReceiver.from_config()
mag = HallSensor.from_config()
mag.calibrate_baseline()            # no magnet nearby

while True:
    now = ticks_ms()
    bits = line.read()
    error = tracker.update(bits, now)        # -1.0 (line left) .. +1.0 (line right)
    event = junctions.update(bits, now)      # None or JunctionEvent
    for s in tofs.values():
        s.poll(now)
    bumpers.update(now)
    ir.process()
    mag.update(now)

    if event is not None and event.kind == "EXIT":
        print(event.shape, event.left, event.right, event.straight)
    if tofs["dist1"].is_closer_than(80):
        pass                                 # object ahead of Distance Sensor 1
    hit = bumpers.consume_hits()             # None, "LEFT", "RIGHT", "BOTH"
    frame = ir.get_frame()                   # None or a list of pulse lengths
    if mag.consume_detection():
        pass                                 # magnetic box
    sleep_ms(5)
```

Junction notes: `JunctionDetector` reports `ENTER` as soon as a branch is
confirmed and `EXIT` once the robot has passed it, with the final
`left` / `right` / `straight` and `shape`. Call `junctions.reset()` after a
turn, because spinning sweeps the sensors over other lines.

---

## 11. Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Viewer: "Cannot open COM..." / mpremote "failed to access" | another program holds the port | disconnect MicroPico, close other viewers / terminals |
| Viewer: "no data - is the code uploaded?" | files missing on the Pico, or `main.py` running | upload the project; press Connect again (the viewer interrupts `main.py`) |
| `AttributeError: module has no attribute ...` after an upload | the Pico still uses an older imported module | soft-reset (Ctrl-D) |
| ToF bus scan `[]` | no power, wrong pins, C/D swapped, SDA tied to VCC | `+` on 36, `-` on GND; dist1: D on 21, C on 22; dist2: D on 14, C on 15; physical 21 must not touch 36; check with test_wiring |
| ToF works, then I2C errors climb and readings stop | loose wire or power glitch | fix the wiring, then Apply in the viewer / restart the test |
| ToF always `None` while raw looks right | range status not 11 | check `range_status`; `require_valid_status=False` disables the check |
| Line pattern inverted | output polarity | `LINE_WHITE_IS_HIGH = False` |
| Line characters in the wrong place | wires do not match the schematic | compare with section 5, or swap the GP numbers in `config.py` |
| Crash always "pressed" | sensor unpowered, DIG/S on the wrong pin | measure DIG/S (3.3 V released); check VCC on 36 |
| Left and right crash swapped | L / R wires swapped | final schematic: L on physical 12, R on 11; swap the wires or `CRASH_LEFT_PIN` / `CRASH_RIGHT_PIN` |
| IR: never any frames | VS on the wrong pin, OUT not on physical 19, remote not 38 kHz | pins 1 OUT, 2 GND, 3 VS; try another remote |
| Magnet detected with no magnet nearby | baseline drift or a magnet nearby during calibration | recalibrate (Apply in the viewer / restart the test) away from magnets; raise `MAG_OFF_MV` |
| `SyntaxError` in `main.py` on the Pico | MicroPython-incompatible syntax | see section 12 |
| Robot turns the wrong way when the line is to one side | motors swapped or inverted | redo step 9 |
| Robot starts by itself | the start/stop crash sensor was bumped | that is the button; keep hands clear of the front |

---

## 12. Known issues and to-do

Main program
1. Measure on the table / robot and update `main.py`: `SENSOR_AHEAD_M`
   (wheel axle -> line sensors), the distances in `CLOSE_AFTER_TURN`
   (start box edges, pad junction spacing) and `RUNG_SPACING_M`.
2. Measure `MAG_NORTH_SIGN` and `SLOT_OCCUPIED_MM` (robot turned into a
   slot, empty / full); set `MAG_POLE_LEVEL` to the task rule (`config.py`,
   "Task sensors"). (`FRONT_TOF` / `GRAB_TOF` were swapped on 2026-10-26:
   dist2 faces forward.)
3. Leaving a drop slot with an "about" turn: the wheels are then on the
   junction, so the sensors start past it and the robot can drive on into
   the opposite rung (seen in the simulator). To be handled by the custom
   drop-off function.
4. The gripper (2026-10-26): tune `GRAB_STOP_MM`, `GRAB_APPROACH_SPEED`,
   `GRIPPER_RELEASE_DEG` on the table and measure the new grab angle
   (`SERVO_GRAB_ANGLE`). The turn into a pick-up box must end on its line,
   or the drive up to the box goes skew. Distance sensor dist1 (now the grab
   sensor, I2C0 GP16 / GP17) does not answer since the hardware change
   (OSError 110): check its wiring. In the simulator the pads were moved to
   0.35 m from the side lines, because the box is ahead of the robot after
   the turn.
5. A turn that never finds a line keeps spinning (press the button).

Hardware / schematic
5. **Schematic error: Distance Sensor 1 SDA is drawn joined to VCC** (section
   5). Fix the drawing and make sure the real wiring does not copy it.
6. A start button is not in the schematic; GP12 (physical 16) is the only free
   pin left.
7. GP0-GP3 (motor driver 1) are not in the schematic; GP2 / GP3 are inferred
   from the department examples.
8. A ToF sensor that loses power is not re-initialised automatically yet.

---

## 13. Development notes

- Code, comments and docs are in English.
- MicroPython is not CPython. Avoid: `match`, type hints, `dataclasses`,
  `typing`, `collections.deque` without a maxlen, `str.ljust/rjust/zfill`,
  numeric literal underscores, indexing dict views.
- Time: use `utime.ticks_ms()` and always compare with `ticks_diff()`.
- Before pushing: `python -m sim.check` (all PASS) and check that changed
  files compile for MicroPython:

  ```
  pip install mpy-cross
  python -m mpy_cross -o NUL main.py
  ```

  (use `-o /dev/null` on macOS / Linux)
- Keep pins in `config.py` only, keep them in step with the schematic, and
  run `config.validate()` and `config.warnings()` after changing them.
