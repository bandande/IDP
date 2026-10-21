"""
Fake `machine` module for running the robot code on a PC against the
simulator (only what the robot code uses).

Pin  - outputs are reported to the world (motor DIR pins); inputs ask the
       world for their level (line sensors, crash sensors). IRQ handlers
       fire on edges the world drives with sim_set().
PWM  - duty is reported to the world (motor speed).
ADC, I2C, SoftI2C - present so imports work; no devices are attached.
Every pin read costs a little simulated time, like on the Pico.
"""

from sim.fakes import _bus

# Simulated time spent by one Pin.value() read (microseconds).
PIN_READ_COST_US = 30


class Pin:
    IN = 0
    OUT = 1
    OPEN_DRAIN = 2
    PULL_UP = 1
    PULL_DOWN = 2
    IRQ_FALLING = 4
    IRQ_RISING = 8

    _registry = {}

    def __init__(self, id, mode=-1, pull=-1, value=None, **kwargs):
        self.id = id
        self._mode = mode
        self._pull = pull
        # Creating a Pin object again does not change the level on the wire:
        # keep the level of an existing pin. A new input with a pull-up idles
        # HIGH (e.g. the IR receiver, buttons).
        old = Pin._registry.get(id)
        if old is not None:
            self._value = old._value
        else:
            self._value = 1 if (mode == Pin.IN and pull == Pin.PULL_UP) else 0
        self._handler = None
        self._trigger = 0
        Pin._registry[id] = self
        if value is not None:
            self.value(value)

    def init(self, mode=-1, pull=-1, value=None, **kwargs):
        if mode != -1:
            self._mode = mode
        if pull != -1:
            self._pull = pull
        if value is not None:
            self.value(value)

    def value(self, v=None):
        if v is None:
            world = _bus.world
            if world is not None:
                world.advance(PIN_READ_COST_US)
                level = world.pin_input(self.id)
                if level is not None:
                    self._value = level
            return self._value
        self._value = 1 if v else 0
        if _bus.world is not None:
            _bus.world.pin_output(self.id, self._value)
        return None

    def __call__(self, v=None):
        return self.value(v)

    def on(self):
        self.value(1)

    def off(self):
        self.value(0)

    def high(self):
        self.value(1)

    def low(self):
        self.value(0)

    def toggle(self):
        self.value(0 if self._value else 1)

    def irq(self, handler=None, trigger=IRQ_FALLING | IRQ_RISING, priority=1, wake=None, hard=False):
        self._handler = handler
        self._trigger = trigger
        return self

    def sim_set(self, v):
        """Drive an input from the simulator; fires the IRQ handler on a matching edge."""
        v = 1 if v else 0
        old = self._value
        self._value = v
        if self._handler is not None and v != old:
            if (v and self._trigger & Pin.IRQ_RISING) or (not v and self._trigger & Pin.IRQ_FALLING):
                self._handler(self)

    @classmethod
    def get(cls, id):
        return cls._registry.get(id)


class PWM:
    _registry = {}

    def __init__(self, dest, freq=None, duty_u16=None, duty_ns=None, invert=False):
        self.pin_id = getattr(dest, "id", dest)
        self._freq = 1000
        self._duty = 0
        PWM._registry[self.pin_id] = self
        if freq is not None:
            self.freq(freq)
        if duty_u16 is not None:
            self.duty_u16(duty_u16)

    def init(self, freq=None, duty_u16=None, duty_ns=None, invert=False):
        if freq is not None:
            self.freq(freq)
        if duty_u16 is not None:
            self.duty_u16(duty_u16)

    def freq(self, f=None):
        if f is None:
            return self._freq
        self._freq = int(f)
        return None

    def duty_u16(self, d=None):
        if d is None:
            return self._duty
        self._duty = max(0, min(65535, int(d)))
        if _bus.world is not None:
            _bus.world.pwm_output(self.pin_id, self._duty)
        return None

    def deinit(self):
        self.duty_u16(0)

    @classmethod
    def get(cls, pin_id):
        return cls._registry.get(pin_id)


class ADC:
    CORE_TEMP = 4

    def __init__(self, pin):
        self.pin_id = getattr(pin, "id", pin)

    def read_u16(self):
        world = _bus.world
        if world is not None:
            v = world.adc_input(self.pin_id)
            if v is not None:
                return v
        return 32768


class I2C:
    def __init__(self, id=-1, scl=None, sda=None, freq=400000, timeout=50000):
        self.id = id

    def scan(self):
        return []

    def _none(self, *args, **kwargs):
        raise OSError(19, "simulator: no I2C device")

    readfrom_mem = readfrom_mem_into = writeto_mem = readfrom = writeto = readfrom_into = _none


class SoftI2C(I2C):
    def __init__(self, scl=None, sda=None, freq=400000, timeout=50000):
        I2C.__init__(self, -1, scl, sda, freq, timeout)


def disable_irq():
    return 0


def enable_irq(state=None):
    pass


def freq(f=None):
    return 150000000 if f is None else None


def unique_id():
    return b"SIMULATR"


def reset():
    raise SystemExit("machine.reset() called in the simulator")


def soft_reset():
    raise SystemExit("machine.soft_reset() called in the simulator")
