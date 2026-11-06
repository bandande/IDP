from machine import Pin, ADC
from utime import ticks_ms, sleep_ms

from sensors.filters import Debouncer

DIGITAL = "digital"
ANALOG = "analog"
ADC_PINS = (26, 27, 28)
_U16_TO_MV = 3300.0 / 65535.0


class FieldDetector:
    """Pure logic for an analog Hall signal: baseline, field, hysteresis, polarity."""

    def __init__(self, on_mv=150, off_mv=100, baseline_mv=None):
        if off_mv > on_mv:
            raise ValueError("off_mv must be <= on_mv")
        self.on_mv = on_mv
        self.off_mv = off_mv
        self.baseline_mv = baseline_mv
        self.detected = False
        self.field_mv = 0
        self.polarity = 0

    def update(self, mv):
        """Feed one reading in mV. Returns the detected state."""
        if self.baseline_mv is None:
            self.baseline_mv = mv          # first reading becomes the baseline
        self.field_mv = int(mv - self.baseline_mv)
        size = abs(self.field_mv)
        if size >= self.on_mv:
            self.detected = True
        elif size <= self.off_mv:
            self.detected = False
        if self.detected:
            self.polarity = 1 if self.field_mv > 0 else -1
        else:
            self.polarity = 0
        return self.detected


class HallSensor:
    """One 3-wire Hall sensor, digital or analog.

    mode: "digital" or "analog" (analog needs GP26/27/28).
    active_low: digital only - True if the output goes LOW near a magnet.
    debounce_ms: digital only.
    adc_samples: analog only - readings averaged per update.
    on_mv / off_mv / baseline_mv: analog only, see FieldDetector.
    """

    def __init__(self, pin, mode=DIGITAL, active_low=True, debounce_ms=10, adc_samples=8,
                 on_mv=150, off_mv=100, baseline_mv=None):
        if mode not in (DIGITAL, ANALOG):
            raise ValueError("mode must be 'digital' or 'analog'")
        if mode == ANALOG and pin not in ADC_PINS:
            raise ValueError("analog mode needs an ADC pin: GP26, GP27 or GP28")
        if adc_samples < 1:
            raise ValueError("adc_samples must be >= 1")
        self.mode = mode
        self.pin_number = pin
        self.active_low = active_low
        self.adc_samples = adc_samples
        self.mv = None
        self._pending = 0
        self.detections = 0
        if mode == DIGITAL:
            # Many Hall chips have an open-drain output; the pull-up is harmless otherwise.
            self._pin = Pin(pin, Pin.IN, Pin.PULL_UP)
            self._debouncer = Debouncer(debounce_ms)
            self._field = None
        else:
            self._adc = ADC(Pin(pin))
            self._field = FieldDetector(on_mv, off_mv, baseline_mv)

    @classmethod
    def from_config(cls):
        import config
        return cls(config.MAG_PIN, config.MAG_MODE, config.MAG_ACTIVE_LOW, config.MAG_DEBOUNCE_MS,
                   on_mv=config.MAG_ON_MV, off_mv=config.MAG_OFF_MV)

    def read_mv(self):
        """Averaged ADC reading in mV (analog mode only)."""
        total = 0
        for _ in range(self.adc_samples):
            total += self._adc.read_u16()
        return int(total * _U16_TO_MV / self.adc_samples)

    def raw(self):
        """Digital: pin level 0/1. Analog: averaged reading in mV."""
        if self.mode == DIGITAL:
            return self._pin.value()
        return self.read_mv()

    def calibrate_baseline(self, samples=50):
        """Analog: average the output with NO magnet nearby and use it as zero field.
        Blocking (about samples ms). Returns the baseline in mV."""
        if self.mode != ANALOG:
            return None
        total = 0
        for _ in range(samples):
            total += self.read_mv()
            sleep_ms(1)
        self._field.baseline_mv = total // samples
        return self._field.baseline_mv

    def update(self, now_ms=None):
        """Call every main-loop iteration. Returns True while a magnet is detected."""
        was = self.detected
        if self.mode == DIGITAL:
            if now_ms is None:
                now_ms = ticks_ms()
            level = self._pin.value()
            self.mv = None
            self._debouncer.update(level == (0 if self.active_low else 1), now_ms)
        else:
            self.mv = self.read_mv()
            self._field.update(self.mv)
        if self.detected and not was:
            self.detections += 1
            self._pending += 1
        return self.detected

    @property
    def detected(self):
        if self.mode == DIGITAL:
            return self._debouncer.state
        return self._field.detected

    @property
    def field_mv(self):
        """Analog: field relative to the baseline in mV (None in digital mode)."""
        return None if self._field is None else self._field.field_mv

    @property
    def polarity(self):
        """Analog: +1 / -1 for the two poles while detected, 0 otherwise (None in digital mode)."""
        return None if self._field is None else self._field.polarity

    @property
    def baseline_mv(self):
        return None if self._field is None else self._field.baseline_mv

    def consume_detection(self):
        """True once for every new detection since the previous call."""
        if self._pending:
            self._pending -= 1
            return True
        return False
