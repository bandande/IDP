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
