'''
Interface for the push button

ADOPTED (2026-10-23) from the partner's main (91d3af5 / 64aa5b2), with the
fixes marked FIX below. The button is on config.PUSH_BUTTON_PIN (GP12). It is
used as the start / stop button when config.START_STOP_SENSOR = "push"
(see control/start_stop.py); the default is still the left crash sensor.
'''

from machine import Pin
from utime import ticks_ms, ticks_us, ticks_diff
from sensors.filters import Debouncer

class PushButton:
    '''push button with debouncing

    pin: GPIO number
    active_low: when the button is pressed, is the signal low or high (True for low)
    '''

    def __init__(self, pin, active_low=True, debounce_ms=15, name='push button'):
        self.name=name
        self._active_level = 0 if active_low == True else 1
        self._debouncer = Debouncer(debounce_ms)
        if active_low == True:
            self._pin = Pin(pin, Pin.IN, Pin.PULL_UP)
        else:
            self._pin = Pin(pin, Pin.IN, Pin.PULL_DOWN)
        self.held = False
        self.hit = 0
        self.pending = 0

    # FIX (2026-10-23): `pressed` is now a PROPERTY (read it without brackets),
    # the same as CrashSensor.pressed. StartStopButton.held() reads
    # `self.sensor.pressed` without brackets. While `pressed` was a normal
    # method, that gave the method object itself, which always counts as True,
    # so the pause code thought the button was held for ever: every short
    # press (pause) turned into a long press (stop and reset).
    @property
    def pressed(self):
        '''Debounced pressed state as of last update'''
        return self._debouncer.state

    def raw(self):
        """Current pin level (0 or 1)."""
        return self._pin.value()

    def _add_hit(self):
        self.hit+=1
        self.pending += 1

    def consume_hit(self):
        """True once for every new press since the previous call."""
        # FIX (2026-10-23): the counter is called self.pending (set in
        # __init__ and _add_hit). The old code used self._pending here, which
        # does not exist, so the first press crashed with AttributeError.
        if self.pending:
            self.pending -= 1
            return True
        return False

    def update(self, now_ms=None):
        '''Call every main loop iteration, returns debounced pressed state'''
        if now_ms == None:
            now_ms = ticks_ms()
        active = self._pin.value() == self._active_level
        pressed = self._debouncer.update(active, now_ms)
        if self._debouncer.rose:
            self._add_hit()
        return pressed

    # ADDED (2026-10-23): StartStopButton.deinit() calls sensor.deinit(), which
    # this class did not have. Nothing to release (no interrupt is used), so it
    # only clears the press counter.
    def deinit(self):
        self.pending = 0
