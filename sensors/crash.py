"""
Interface for the 2 x DFRobot SEN0138 crash sensors (SEN0138-L / SEN0138-R).

Device facts
  * A micro-switch with a roller lever on a small board with an on-board
    pull-up resistor and a status LED. Pins: 1 = digital out, 2 = VCC, 3 = GND.
  * Output HIGH (at the supply voltage) when released, LOW when pressed
    (DFRobot's example code treats LOW as "pressed").
  * The team schematic powers both boards from 3V3_OUT, so the output is at
    most 3.3 V and goes straight to the Pico pin (left GP6, right GP7). DFRobot
    rates the board for 5V, but the switch and its pull-up work at 3.3 V; only
    the LED is dimmer.
  * !!! If the boards are ever powered from 5V, the released output is 5V and
    a voltage divider is REQUIRED (e.g. 10k from the output to the pin, 20k
    from the pin to GND). RP2040 GPIOs are not 5V tolerant and the RP2350
    only tolerates 5V while it is powered.
  * On early RP2350 chips (A2 stepping, erratum E9) an input that is only
    weakly pulled down can stay at about 2.2 V and read HIGH, so an unplugged
    sensor may read "released". Always do a press test after wiring.

Data processing
  * Debounced level: the state only changes after the pin has been stable
    for debounce_ms (CRASH_DEBOUNCE_MS).
  * Short-bump capture: a hard IRQ on both edges remembers when a press
    started and when it last released. If a bump starts and ends between two
    update() calls (the main loop was busy), it still counts as a hit when it
    lasted at least min_press_ms. Shorter glitches (e.g. motor noise) are
    ignored.
  * consume_hit() reports each press exactly once. BumperPair combines both
    sensors; both pressed together means square contact with a wall / object.

Usage
    from sensors.crash import BumperPair
    bumpers = BumperPair.from_config()
    while True:
        bumpers.update()
        hit = bumpers.consume_hits()      # None, "LEFT", "RIGHT" or "BOTH"
"""

import machine
import micropython
from machine import Pin
from utime import ticks_ms, ticks_us, ticks_diff

from sensors.filters import Debouncer

micropython.alloc_emergency_exception_buf(100)

LEFT = "LEFT"
RIGHT = "RIGHT"
BOTH = "BOTH"


def _side_code(left, right):
    if left and right:
        return BOTH
    if left:
        return LEFT
    if right:
        return RIGHT
    return None


class CrashSensor:
    """One crash switch with debouncing and short-press capture.

    pin: GPIO number. active_low: True for the SEN0138 (pressed -> 0 V).
    pull: optional Pin.PULL_UP / Pin.PULL_DOWN (None: the sensor's own pull-up drives the pin).
    use_irq: False disables the short-press capture (debounced polling only).
    """

    def __init__(self, pin, active_low=True, debounce_ms=15, min_press_ms=3, name="crash",
                 use_irq=True, pull=None):
        self.name = name
        self.min_press_us = min_press_ms * 1000
        self._active_level = 0 if active_low else 1
        if pull is None:
            self._pin = Pin(pin, Pin.IN)
        else:
            self._pin = Pin(pin, Pin.IN, pull)
        self._debouncer = Debouncer(debounce_ms)
        self.hits = 0
        self._pending = 0
        # State shared with the IRQ handler. Every attribute exists before the
        # IRQ is enabled, so the handler only overwrites ints/bools (no allocation).
        self._in_press = False
        self._released = False
        self._press_us = 0
        self._release_us = 0
        self._counted = False
        self._irq = None
        if use_irq:
            self._irq = self._pin.irq(handler=self._isr, trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING,
                                      hard=True)

    def _isr(self, pin):
        # Hard IRQ: integers and bools only.
        t = ticks_us()
        if pin.value() == self._active_level:
            if not self._in_press:
                self._in_press = True
                self._press_us = t
        elif self._in_press:
            self._released = True
            self._release_us = t

    def raw(self):
        """Current pin level (0 or 1)."""
        return self._pin.value()

    def active(self):
        """True if the switch is pressed right now (no debouncing)."""
        return self._pin.value() == self._active_level

    @property
    def pressed(self):
        """Debounced pressed state (as of the last update())."""
        return self._debouncer.state

    def update(self, now_ms=None):
        """Call every main-loop iteration. Returns the debounced pressed state."""
        if now_ms is None:
            now_ms = ticks_ms()
        irq_state = machine.disable_irq()
        active = self._pin.value() == self._active_level
        episode_done = self._released and not active
        press_us = self._press_us
        release_us = self._release_us
        if episode_done:
            self._in_press = False
            self._released = False
        machine.enable_irq(irq_state)

        pressed = self._debouncer.update(active, now_ms)
        if self._debouncer.rose:
            self._add_hit()
            self._counted = True
        if episode_done:
            # The IRQ saw a complete press. Count it only if the debouncer missed it.
            if not self._counted and ticks_diff(release_us, press_us) >= self.min_press_us:
                self._add_hit()
            self._counted = False
        elif not pressed and not self._in_press:
            self._counted = False
        return pressed

    def _add_hit(self):
        self.hits += 1
        self._pending += 1

    def consume_hit(self):
        """True once for every new press since the previous call."""
        if self._pending:
            self._pending -= 1
            return True
        return False

    def reset(self):
        """Forget pending hits and the debounced state."""
        irq_state = machine.disable_irq()
        self._in_press = False
        self._released = False
        machine.enable_irq(irq_state)
        self._debouncer.reset(False)
        self._pending = 0
        self._counted = False

    def deinit(self):
        """Disable the IRQ."""
        if self._irq is not None:
            self._pin.irq(handler=None)
            self._irq = None


class BumperPair:
    """Left and right crash sensors used together."""

    def __init__(self, left, right):
        self.left = left
        self.right = right

    @classmethod
    def from_config(cls):
        import config
        return cls(CrashSensor(config.CRASH_LEFT_PIN, config.CRASH_ACTIVE_LOW,
                               config.CRASH_DEBOUNCE_MS, name="left"),
                   CrashSensor(config.CRASH_RIGHT_PIN, config.CRASH_ACTIVE_LOW,
                               config.CRASH_DEBOUNCE_MS, name="right"))

    def update(self, now_ms=None):
        if now_ms is None:
            now_ms = ticks_ms()
        self.left.update(now_ms)
        self.right.update(now_ms)
        return self.which()

    def any_pressed(self):
        return self.left.pressed or self.right.pressed

    def both_pressed(self):
        return self.left.pressed and self.right.pressed

    def which(self):
        """Current debounced contact: None, "LEFT", "RIGHT" or "BOTH"."""
        return _side_code(self.left.pressed, self.right.pressed)

    def consume_hits(self):
        """New hits since the previous call: None, "LEFT", "RIGHT" or "BOTH"."""
        left = self.left.consume_hit()
        right = self.right.consume_hit()
        return _side_code(left, right)

    def reset(self):
        self.left.reset()
        self.right.reset()
