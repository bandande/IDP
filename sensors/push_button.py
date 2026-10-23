'''
Interface for the push button
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
        if self._pending:
            self._pending -= 1
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
