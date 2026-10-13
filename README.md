# IDP

Cambridge Part IB Integrated Design Project: a line-following robot on a
Raspberry Pi Pico 2 (RP2350) running MicroPython.

## Repository layout

| Path | Runs on | Contents |
|---|---|---|
| `main.py`, `functions.py`, `MAP.py`, `constants.py` | Pico | main loop, navigation and course map |
| `config.py` | Pico | every sensor pin and tuning constant; `validate()` checks for pin clashes |
| `sensors/` | Pico | sensor interfaces and data processing (one module per sensor, see below) |
| `libs/VL53L0X.py` | Pico | VL53L0X driver from the department template |
| `tests/` | Pico | on-robot test scripts `test_*.py`, and `stream_sensors.py` used by the viewer |
| `tools/sensor_viewer.py` | PC | live sensor viewer window (not uploaded to the Pico) |

| Module | Sensor | Main classes |
|---|---|---|
| `sensors/line_sensor.py`, `sensors/line_processing.py` | 4 x DFRobot SEN0017 line sensors | `LineSensorArray`, `LineTracker`, `JunctionDetector` |
| `sensors/tof.py` | 2 x VL53L0X distance sensors | `ToFSensor`, `tof_from_config()` |
| `sensors/crash.py` | 2 x DFRobot SEN0138 crash sensors | `CrashSensor`, `BumperPair` |
| `sensors/ir_receiver.py` | Vishay TSOP4838 IR receiver | `IRReceiver`, `decode_nec()`, `match_template()` |
| `sensors/magnetic.py` | 3-wire Hall sensor (analog) | `HallSensor` |
| `sensors/filters.py` | shared helpers | `MedianFilter`, `Debouncer`, `Hysteresis` |

Every module starts with a docstring describing the device, its wiring and a
usage example. Every sensor object is created from `config.py` with
`from_config()` and is polled without blocking from the main loop.

## Wiring

Code uses GP numbers; the jumpers on the robot board are labelled with the
physical pin numbers. `config.physical_pin(gp)` converts.

| Sensor | Signal | GP | Physical pin | Power |
|---|---|---|---|---|
| Line outer-left / inner-left / inner-right / outer-right | OUT | 18 / 19 / 20 / 21 | 24 / 25 / 26 / 27 | 3V3 (pin 36) |
| ToF front (`+ - C D`) | D = SDA, C = SCL | 8, 9 | 11, 12 | 3V3 (pin 36) |
| ToF side (`+ - C D`) | D = SDA, C = SCL | 10, 11 | 14, 15 | 3V3 (pin 36) |
| Crash left / right | OUT via divider | 16 / 17 | 21 / 22 | 5V only |
| IR receiver TSOP4838 (1 OUT, 2 GND, 3 VS) | OUT | 22 | 29 | 3V3 (pin 36) |
| Magnetic (Hall, analog) | OUT | 26 (ADC0) | 31 | 3V3 (pin 36) |

- The crash sensors output 5V: put a divider (10k from the output to the pin,
  20k from the pin to GND) in between. Pico GPIOs must not see 5V.
- Pins already used by the robot board: GP0-GP7 (motor drivers), GP13 and
  GP15 (servo headers).

## Testing

On the robot: upload the project with MicroPico ("Upload project to Pico"),
open a `tests/test_*.py` file and use "Run current file on Pico".

Live viewer on the PC (close MicroPico's connection first, only one program
can use the serial port):

    pip install pyserial
    python tools/sensor_viewer.py

Tabs: Overview (all sensors and raw pin levels), Distance (ToF curve,
I2C scan, offset calibration) and Magnetic (Hall voltage curve).

## Status (2026-10-13)

Tested on the robot: ToF front (no I2C errors over 6 minutes, about 35
readings/s, 24-829 mm), magnetic sensor (analog output, thresholds set in
`config.py`), two line sensors. Not yet tested on hardware: ToF side, crash
sensors, IR receiver.
