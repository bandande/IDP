# IDP Robot

Cambridge Part IB Integrated Design Project: a line-following robot on a
Raspberry Pi Pico 2 (MicroPython). It collects boxes from the pick-up points
and delivers each one to a free rack slot, using the box colour to choose the
side and the box's magnet pole to choose the level.

## Running

1. Upload the code to the Pico with the VS Code MicroPico extension
   (`sim/` and `tools/` are PC-only and are not uploaded).
2. Power on the robot in the start box: `main.py` starts by itself and the
   yellow LED turns on.
3. Press the left crash sensor to start. A short press pauses or continues;
   a long press stops and resets.

Every run's output is saved on the Pico as `print_log.txt`.

## Layout

| Path | Contents |
|---|---|
| `main.py` | Entry point and main loop |
| `functions.py` | Robot state machine, route planning, line following and turns |
| `MAP.py` | The arena as a graph of named junctions |
| `config.py` | Pins and sensor settings |
| `constants.py` | Speeds, timings and distances |
| `motor.py` | Drive motor driver |
| `actuators/` | Servos, gripper movements and status LEDs |
| `control/` | Pick-up sequence, task sensors and start/stop button (plus two helpers used only by tests) |
| `sensors/` | Line, distance, magnet, IR and crash sensor drivers, plus filters |
| `libs/` | VL53L0X distance sensor driver |
| `tests/` | Bench tests for each part, run from the REPL |
| `sim/` | PC simulator (written for an earlier version of the code) |
| `tools/` | PC live sensor viewer |

<sub>AI tools were used only to help find and fix bugs, to help understand the code, and to write comments.</sub>
