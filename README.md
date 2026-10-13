# IDP - Sensor Bench Guide

Cambridge Part IB Integrated Design Project. A line-following robot on a
Raspberry Pi Pico 2 (RP2350) running MicroPython. This README is the guide
for the sensor bench session: what is in the repository, how every sensor is
wired, how to get the code onto the Pico, and a step-by-step test plan with
pass criteria, calibration sheet and troubleshooting.

---

## 1. Status (2026-10-13)

| Sensor | Count | Code | Tested on the robot |
|---|---|---|---|
| Line tracking, DFRobot SEN0017 | 4 | `sensors/line_sensor.py`, `sensors/line_processing.py` | 2 of 4, on temporary test pins (see section 12) |
| Distance (ToF), VL53L0X | 2 | `sensors/tof.py` | front: yes (6 min, 0 I2C errors, about 35 readings/s, 24-829 mm); side: not yet |
| Crash, DFRobot SEN0138 L/R | 2 | `sensors/crash.py` | not yet |
| IR receiver, Vishay TSOP4838 | 1 | `sensors/ir_receiver.py` | not yet |
| Magnetic (Hall, analog output) | 1 | `sensors/magnetic.py` | yes (analog, thresholds set) |

The board: `Raspberry Pi Pico2 with RP2350`, MicroPython v1.28.0, appears as a
USB serial port (COM8 on the laptop used so far, USB ID 2E8A:0005).

---

## 2. Bring / install before the session

Hardware
- Robot board with the Pico 2, USB cable
- Multimeter (needed before the crash sensors are connected)
- Resistors for the crash sensor dividers: 2 x 10k and 2 x 20k (or similar, see section 5)
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
| `main.py` | Pico | main loop (work in progress, see section 12). The Pico runs it automatically at every boot. |
| `functions.py`, `MAP.py`, `constants.py` | Pico | navigation: `Robot` state, route finding on the course map, turn helpers |
| `config.py` | Pico | every sensor pin and tuning constant; `validate()` checks for pin clashes; `physical_pin(gp)` |
| `sensors/` | Pico | one module per sensor: hardware interface + data processing |
| `libs/VL53L0X.py` | Pico | VL53L0X driver from the department template (unchanged) |
| `tests/test_*.py` | Pico | on-robot test scripts, one per sensor, numbered in the recommended order |
| `tests/stream_sensors.py` | Pico | streams every reading to the PC as JSON lines for the viewer |
| `tools/sensor_viewer.py` | PC | live viewer window. Not uploaded to the Pico (excluded in `.vscode/settings.json`). |
| `.micropico`, `.vscode/` | PC | MicroPico project settings |

Sensor modules

| Module | Main API |
|---|---|
| `sensors/line_sensor.py` | `LineSensorArray.from_config()`, `.read()` -> 4-bit mask (bit set = white), `.read_raw()` |
| `sensors/line_processing.py` | `LineTracker` (steering error -1..+1, ON_LINE / SEARCHING / LOST), `JunctionDetector` (ENTER / EXIT events with shape FOUR_JUNCTION / T_JUNCTION / CORNER, the names used in `MAP.py`), `bits_to_str()` |
| `sensors/tof.py` | `tof_from_config()` -> `{"front": ToFSensor, "side": ToFSensor}`; `.poll()`, `.distance_mm` (None = no target / stale / unavailable), `.raw_mm`, `.range_status`, `.is_closer_than(mm)`, `.begin()` |
| `sensors/crash.py` | `BumperPair.from_config()`, `.update()`, `.which()` -> None / "LEFT" / "RIGHT" / "BOTH", `.consume_hits()` |
| `sensors/ir_receiver.py` | `IRReceiver.from_config()`, `.process()`, `.signal_present()`, `.get_frame()`; `decode_nec(frame)`, `match_template(frame, template)`, `frame_signature(frame)` |
| `sensors/magnetic.py` | `HallSensor.from_config()`, `.calibrate_baseline()`, `.update()`, `.detected`, `.consume_detection()`, `.field_mv`, `.polarity` |
| `sensors/filters.py` | `MedianFilter`, `Debouncer`, `Hysteresis` |

Every module starts with a docstring that explains the device, its wiring, the
data processing and a usage example.

---

## 4. Reading the Pico pinout

- Two numbering schemes. **Code uses GP numbers** (`Pin(18)` is GP18). **The
  jumpers on the robot board are labelled with physical pin numbers 1-40.**
  They are different: GP18 is physical pin 24. `config.physical_pin(18)`
  returns 24, and every pin in `config.py` has both numbers in its comment.
- Physical numbering: board seen from the top, USB socket at the top. Pin 1
  is top-left, numbers go down the left side to 20, continue at the bottom
  right with 21 and go up the right side to 40.
- Every fifth pin is a GND: 3, 8, 13, 18, 23, 28, 33 (AGND), 38.
- Power pins: 36 = 3V3(OUT), the 3.3 V supply for sensors. 39 = VSYS.
  40 = VBUS, 5 V **only while USB is plugged in**. Do not connect anything to
  37 (3V3_EN), 30 (RUN) or 35 (ADC_VREF).
- Only GP26, GP27, GP28 (physical 31, 32, 34) can read analog voltages.
- I2C: SDA and SCL must belong to the same bus. I2C0 on GP8/GP9 and I2C1 on
  GP10/GP11 are used here (one ToF sensor per bus).
- **All GPIO pins are 3.3 V. Never put 5 V on a GPIO pin.**

### Header map with this project's assignments

```
 Left side                                       Right side
 phys  GP    use                                 phys  GP     use
  1    GP0   motor driver 1 #1 DIR   (board)      21   GP16   crash LEFT (via divider)
  2    GP1   motor driver 1 #1 PWM   (board)      22   GP17   crash RIGHT (via divider)
  3    GND                                        23   GND
  4    GP2   motor driver 1 #2 (assumed)          24   GP18   line outer-left
  5    GP3   motor driver 1 #2 (assumed)          25   GP19   line inner-left
  6    GP4   motor driver 2 #1 DIR   (board)      26   GP20   line inner-right
  7    GP5   motor driver 2 #1 PWM   (board)      27   GP21   line outer-right
  8    GND                                        28   GND
  9    GP6   motor driver 2 #2 PWM   (board)      29   GP22   IR receiver OUT
 10    GP7   motor driver 2 #2 DIR   (board)      30   RUN    do not use
 11    GP8   ToF front  D (SDA)                   31   GP26   magnetic sensor OUT (ADC0)
 12    GP9   ToF front  C (SCL)                   32   GP27   free (ADC1)
 13    GND   ToF ground                           33   AGND   magnetic sensor ground
 14    GP10  ToF side   D (SDA)                   34   GP28   free (ADC2)
 15    GP11  ToF side   C (SCL)                   35   ADC_VREF  do not use
 16    GP12  free (dept. example: start button)   36   3V3(OUT)  sensor power 3.3 V
 17    GP13  servo 2 header (assumed)             37   3V3_EN    do not use
 18    GND                                        38   GND
 19    GP14  free                                 39   VSYS
 20    GP15  servo 1 header  (board)              40   VBUS   5 V only with USB
```

"(board)" pins are wired on the robot board (from the department example
code); "(assumed)" ones are inferred and should be confirmed.

---

## 5. Wiring reference

Do all wiring with the USB cable unplugged.

| Sensor | Sensor pins | Connect to (physical pin) | Extra parts |
|---|---|---|---|
| Line sensor x4 (3 wires) | red +, black -, green signal | + -> 36, - -> any GND, signal -> 24 / 25 / 26 / 27 (left to right) | none |
| ToF front (`+ - C D`) | + / - / C = SCL / D = SDA | + -> 36, - -> 13, C -> 12, D -> 11 | none |
| ToF side (`+ - C D`) | + / - / C = SCL / D = SDA | + -> 36, - -> 13, C -> 15, D -> 14 | none |
| IR receiver TSOP4838 | lens facing you, legs down: 1 OUT, 2 GND, 3 VS | OUT -> 29, GND -> any GND, VS -> 36 | optional supply filter (about 100 ohm in series with VS, a few uF to GND) if noisy near the motors |
| Magnetic sensor (3 wires) | +, -, signal | + -> 36, - -> 33, signal -> 31 | none |
| Crash sensor x2 | 1 OUT, 2 VCC, 3 GND | VCC -> robot board 5 V rail, GND -> GND, OUT -> divider -> 21 (left) / 22 (right) | **voltage divider, required** |

Crash sensor divider (one per sensor). The sensor needs 5 V and its output
idles at 5 V, which would damage the Pico:

```
 sensor OUT --[10k]--+--> Pico pin (physical 21 or 22)
                     |
                   [20k]
                     |
                    GND
```

Measure the divider midpoint before connecting it to the Pico: about
2.2-3.3 V released, about 0 V pressed. Never more than 3.3 V.

Notes
- Only 4 wires on the ToF boards means no XSHUT pin, so both sensors keep
  address 0x29. That is why each one has its own I2C bus.
- Follow the letters printed on each board, not the wire colours.
- Total 3.3 V load (4 line + 2 ToF + IR + Hall) is about 120 mA, within what 3V3(OUT) can supply.

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
- The Pico runs `main.py` at every boot and after every soft reset. While
  `main.py` is unfinished it stops with an error (section 12), which is
  harmless: the REPL is then free for the tests. Once `main.py` contains a
  real loop, press Ctrl-C to stop it before running a test.

---

## 7. Bench test procedure

Go in this order and only wire the sensors needed for each step. Each step
says what "pass" looks like. Write the results into the sheet in section 9.

### Step 0 - before power-up
- [ ] Every sensor + wire goes to 36 (3.3 V), except the crash sensors (5 V).
- [ ] Nothing 5 V reaches a GPIO pin.
- [ ] `config.validate()` reports no problems (step 1 prints it).

### Step 1 - wiring check: `tests/test_wiring.py`
Prints the config check, scans both ToF I2C buses and prints the raw level of
every sensor pin once per second.
- Pass: `config.validate(): OK`; each connected ToF bus shows `[41]` (0x29);
  pin levels change when you act on a sensor.

### Step 2 - line sensors: `tests/test_line_sensors.py`
Mount the sensors 1-2 cm above the floor, in a row across the robot.
1. Over black floor the pattern is `....`.
2. Put white tape under each sensor in turn: the matching character becomes
   `#`, left to right. Wrong position -> fix the pin order in `config.py`.
   Everything inverted -> set `LINE_WHITE_IS_HIGH = False`.
3. Robot centred on the line: `.##.` and error `+0.00`. Slide it sideways:
   line to the left of the robot gives a negative error.
4. Push the robot slowly over a junction: `ENTER` then `EXIT` events.
- Pass: all four sensors respond and are in the right order.
- Geometry assumption used by the code: the two inner sensors are closer
  together than the line width (both on white when centred); each outer
  sensor is more than one line width further out. Measure the line width and
  note the spacing you used.
- The outputs are digital (0/1 only); the SEN0017 has no analog output.

### Step 3 - ToF distance sensors: `tests/test_tof.py` (or the viewer, Distance tab)
Peel the protective film off the sensor window.
1. Both sensors show `available True` (else check the wiring, section 11).
2. Move a hand or box between 100 and 500 mm: the distance follows, status 11.
3. Nothing in front: distance `None` (raw 8190/8191, status 4).
4. Calibrate the offset: flat target exactly 100 mm from the window, then in
   the REPL `from tests.test_tof import calibrate_offset; calibrate_offset("front", 100)`
   (or the Calibrate button in the viewer). Put the printed `offset_mm` into
   `config.TOF_SENSORS`. Repeat for "side".
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
1. Build the dividers (section 5) and **measure the midpoint first**:
   about 2.2-3.3 V released, about 0 V pressed.
2. Only then connect to physical 21 (left) and 22 (right).
3. At rest: raw 1, pressed False. Press a lever: raw 0, pressed True, one hit
   per press. A quick tap still counts exactly once.
- Pass: both sensors, correct side, one hit per press.
- An unplugged sensor may read "released" on this chip (RP2350 erratum E9),
  so always do the press test after wiring.

### Step 7 - everything together: `tests/test_all_sensors.py` (or the viewer, Overview tab)
Run with all sensors connected, then again with the drive motors running.
- Pass: no I2C errors, no false crash hits or IR frames while the motors run.

---

## 8. Live viewer (`tools/sensor_viewer.py`)

```
pip install pyserial
python tools/sensor_viewer.py                       # pick the port, press Connect
python tools/sensor_viewer.py --tab tof --connect   # open on the Distance tab
python tools/sensor_viewer.py --tab mag --connect --record log.txt
```

- Close MicroPico's connection first. The viewer stops whatever runs on the
  Pico, soft-resets it and starts `tests/stream_sensors.py`, so the code must
  be uploaded first.
- Wiring row: type GP numbers (not physical numbers) for each sensor group
  and press Apply. Leave a field empty to switch that group off. This lets
  you test any temporary wiring without editing `config.py`. ToF pins always
  come from `config.py`.
- Overview tab: pin monitor (every free header pin; unused pins read a steady
  HIGH because their pull-up is on, a connected sensor makes its pin change),
  line pattern and error, distances, crash, IR.
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
| Line sensor order correct (L -> R) | yes / no | `LINE_*_PIN` |
| Line sensors white = HIGH | yes / no | `LINE_WHITE_IS_HIGH` |
| Line width / inner spacing / outer spacing (mm) | | (geometry check) |
| Line sensor height above floor (mm) | | |
| ToF front offset at 100 mm (mm) | | `TOF_SENSORS[0]["offset_mm"]` |
| ToF side offset at 100 mm (mm) | | `TOF_SENSORS[1]["offset_mm"]` |
| ToF floor visible at mounting height? | yes / no | (mounting) |
| Crash divider voltage released / pressed, left (V) | / | (safety) |
| Crash divider voltage released / pressed, right (V) | / | (safety) |
| Crash pressed = LOW | yes / no | `CRASH_ACTIVE_LOW` |
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
tofs = tof_from_config()            # blocking start-up, about 0.1 s
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
    if tofs["front"].is_closer_than(80):
        pass                                 # object ahead
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
| ToF bus scan `[]` | no power, wrong pins, C/D swapped | `+` on 36, `-` on GND, D on 11 / 14, C on 12 / 15; check with test_wiring |
| ToF works, then I2C errors climb and readings stop | loose wire or power glitch | fix the wiring, then Apply in the viewer / restart the test |
| ToF always `None` while raw looks right | range status not 11 | check `range_status`; `require_valid_status=False` disables the check |
| Line pattern inverted | output polarity | `LINE_WHITE_IS_HIGH = False` |
| Line characters in the wrong place | pin order | swap the GP numbers in `config.py` |
| Crash always "pressed" | sensor unpowered / divider wrong / unplugged | measure the divider midpoint; check the 5 V supply |
| IR: never any frames | VS on the wrong pin, remote not 38 kHz | pins 1 OUT, 2 GND, 3 VS; try another remote |
| Magnet detected with no magnet nearby | baseline drift or a magnet nearby during calibration | recalibrate (Apply in the viewer / restart the test) away from magnets; raise `MAG_OFF_MV` |
| `SyntaxError` in `main.py` on the Pico | MicroPython-incompatible syntax | see section 12 |

---

## 12. Known issues and to-do

Navigation code (`main.py`, `functions.py`). Checked against a real
MicroPython 1.29 interpreter:
1. `main.py` uses `match ... case` (line 25). **MicroPython does not support
   `match`**; the file fails with `SyntaxError`. Rewrite with `if / elif`.
2. `functions.py` uses `deque([self.position])`. MicroPython's deque needs a
   maximum length: `deque((), 100)` and then `.append(self.position)`.
   Because the last two lines of `functions.py` run a route search at import
   time, `import functions` currently fails on the Pico. Move that test code
   under `if __name__ == "__main__":`.
3. `Robot.move_to()` uses `MAP[...].keys()[...]`. Dict views are not
   indexable in CPython either; use
   `for direction, place in MAP[self.position].items(): if place == location: ...`.
4. `reset_map()` rewrites `MAP.py` on the Pico's flash on every run. Consider
   keeping the visited / collected state in memory instead.

Sensors
5. Line sensors are currently on temporary test pins: physical 21 (GP16) and
   17 (GP13). `config.py` expects physical 24-27 (GP18-21), and GP16 is the
   left crash sensor pin. Move the wires before testing the crash sensors.
6. ToF side, crash sensors and IR receiver are not yet tested on hardware.
7. A ToF sensor that loses power is not re-initialised automatically yet.
8. GP2/GP3 (motor driver 1 #2) and GP13 (servo 2) are inferred from the
   department examples. Confirm on the robot board.

---

## 13. Development notes

- Code, comments and docs are in English.
- MicroPython is not CPython. Avoid: `match`, type hints, `dataclasses`,
  `typing`, `collections.deque` without a maxlen, `str.ljust/rjust/zfill`,
  numeric literal underscores, indexing dict views.
- Time: use `utime.ticks_ms()` and always compare with `ticks_diff()`.
- Check that a file compiles for MicroPython before uploading:

  ```
  pip install mpy-cross
  python -m mpy_cross -o NUL main.py
  ```

  (use `-o /dev/null` on macOS / Linux)
- Keep pins in `config.py` only, and run `config.validate()` after changing them.
