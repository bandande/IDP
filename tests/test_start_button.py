"""
Step 8: start / stop button and indicator LEDs (nothing moves).

The button is the crash sensor named in config.START_STOP_SENSOR ("left" =
physical 12 / GP9, "right" = physical 11 / GP8). The LEDs are yellow (GP22),
red (GP27) and green (GP28), switched through the optocoupler.

Checks:
  1. At start every LED lights for half a second: yellow, red, green.
     If one stays dark, check its optocoupler channel and 220 ohm resistors.
  2. Red LED on = STOP. Press the button once: green on = RUN, yellow
     flashes briefly on every press. Press again: back to STOP.
  3. One press = exactly one toggle (no double toggles).
Stop with Ctrl-C.
"""

from utime import sleep_ms, ticks_ms, ticks_diff

import config
from actuators.leds import StatusLeds
from control.start_stop import StartStopButton


def led_self_test(leds, on_ms=500):
    for colour in ("yellow", "red", "green"):
        leds.off()
        leds.set(colour, True)
        print("LED", colour)
        sleep_ms(on_ms)
    leds.off()


def test_start_button(seconds=None):
    leds = StatusLeds.from_config()
    button = StartStopButton.from_config()
    print("button: %s crash sensor; LEDs %s" % (config.START_STOP_SENSOR, config.LED_PINS))
    led_self_test(leds)
    running = False
    presses = 0
    flash_until = None
    leds.show(red=True)
    print("STOP - press the button")
    start = ticks_ms()
    try:
        while seconds is None or ticks_diff(ticks_ms(), start) < seconds * 1000:
            now = ticks_ms()
            if button.pressed(now):
                presses += 1
                running = not running
                flash_until = now
                print("press %d -> %s" % (presses, "RUN" if running else "STOP"))
            leds.show(green=running, red=not running,
                      yellow=flash_until is not None and ticks_diff(now, flash_until) < 150)
            sleep_ms(5)
    finally:
        leds.off()
        button.deinit()


if __name__ == "__main__":
    test_start_button()
