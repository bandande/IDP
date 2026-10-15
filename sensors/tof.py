"""
Interface for the 2 x VL53L0X time-of-flight distance sensors (I2C).

Device facts
  * 940 nm laser ranging sensor; I2C address 0x29 after power-up or reset.
  * Supply 2.6-3.5 V: power it from the Pico 3V3 pin, not 5V.
  * Useful range about 30-1200 mm in the default mode (less on dark or shiny
    targets and in strong sunlight). Remove the protective film from the window.
  * After each measurement register 0x13 (RESULT_INTERRUPT_STATUS) has a
    non-zero low 3 bits. Register 0x14 (RESULT_RANGE_STATUS) holds the device
    range status in bits 6..3 (11 = range complete / valid, 4 = no target).
    The distance in mm is the big-endian 16-bit value at 0x1E (0x14 + 10);
    8190 / 8191 mean "no target" / "signal fail". Writing 0x01 to 0x0B clears
    the flag so the next sample can be detected.
  * Both sensors answer at 0x29, so either put them on different I2C buses
    (default in config.py) or share one bus and use XSHUT pins: hold one sensor
    in reset (XSHUT low), give the other a new address via register 0x8A, then
    release the first. Addresses are lost on power-off / reset, so
    setup_tof_sensors() repeats this at every boot.

Data processing
  * The sensor ranges continuously; poll() only reads a sample when one is
    ready (otherwise it costs one 1-byte I2C read), so it never blocks.
  * RangeProcessor rejects invalid readings (bad status, error codes, beyond
    max_valid_mm), applies the calibration offset and a median filter, so a
    single spike or a single dropout is removed. A majority of "no target"
    samples gives None.
  * distance_mm is None when there is no target, the sensor is unavailable,
    or the newest sample is older than stale_ms.
  * If distance_mm stays None while raw_mm looks right, check range_status;
    require_valid_status=False disables the status check.

Usage
    from sensors.tof import tof_from_config
    tofs = tof_from_config()               # {"dist1": ToFSensor, "dist2": ToFSensor}
    while True:
        for sensor in tofs.values():
            sensor.poll()
        d = tofs["dist1"].distance_mm      # int mm, or None

    For a yes/no "object is near" signal without chatter at the threshold:
        from sensors.filters import Hysteresis
        near = Hysteresis(on_below=80, off_above=100)
        is_near = near.update(tofs["dist1"].distance_mm)
"""

from machine import Pin, I2C, SoftI2C
from utime import ticks_ms, ticks_diff, sleep_ms

from sensors.filters import MedianFilter

DEFAULT_ADDRESS = 0x29
STATUS_VALID = 11
OUT_OF_RANGE_MM = 8190          # 8190 and above are "no target" / error codes

_REG_INTERRUPT_CLEAR = 0x0B
_REG_INTERRUPT_STATUS = 0x13
_REG_RANGE_STATUS = 0x14
_REG_ADDRESS = 0x8A

_buses = {}


def make_i2c(i2c_id, sda, scl, freq=400000):
    """Return an I2C bus object; buses are shared between sensors on the same pins.

    i2c_id 0 or 1 selects a hardware bus, None selects SoftI2C (any pins).
    """
    key = (i2c_id, sda, scl)
    bus = _buses.get(key)
    if bus is None:
        if i2c_id is None:
            bus = SoftI2C(sda=Pin(sda), scl=Pin(scl), freq=freq)
        else:
            bus = I2C(i2c_id, sda=Pin(sda), scl=Pin(scl), freq=freq)
        _buses[key] = bus
    return bus


def set_address(i2c, old_address, new_address):
    """Give the sensor at old_address a new 7-bit address. Returns True on success."""
    try:
        i2c.writeto_mem(old_address, _REG_ADDRESS, bytes([new_address & 0x7F]))
        return True
    except OSError:
        return False


class RangeProcessor:
    """Validates, offsets and median-filters raw VL53L0X readings (pure logic)."""

    def __init__(self, max_valid_mm=1200, median_size=3, offset_mm=0, require_valid_status=True):
        if max_valid_mm <= 0:
            raise ValueError("max_valid_mm must be > 0")
        self.max_valid_mm = max_valid_mm
        self.offset_mm = offset_mm
        self.require_valid_status = require_valid_status
        self._median = MedianFilter(median_size)
        self._no_target = max_valid_mm + 1
        self.sample_count = 0
        self.valid_count = 0
        self.reset()

    def reset(self):
        self._median.reset()
        self.value = None
        self.last_raw = None
        self.last_status = None
        self.last_update_ms = None

    def to_target_mm(self, raw_mm, status):
        """Corrected distance in mm if the reading is a real target, else None."""
        if self.require_valid_status and status != STATUS_VALID:
            return None
        if raw_mm >= OUT_OF_RANGE_MM:
            return None
        mm = raw_mm + self.offset_mm
        if mm > self.max_valid_mm:
            return None
        return mm if mm > 0 else 0

    def update(self, raw_mm, status, now_ms=None):
        """Feed one reading; returns the filtered distance in mm or None (no target)."""
        if now_ms is None:
            now_ms = ticks_ms()
        self.last_raw = raw_mm
        self.last_status = status
        self.last_update_ms = now_ms
        self.sample_count += 1
        mm = self.to_target_mm(raw_mm, status)
        if mm is None:
            mm = self._no_target
        else:
            self.valid_count += 1
        filtered = self._median.update(mm)
        self.value = None if filtered > self.max_valid_mm else filtered
        return self.value

    def age_ms(self, now_ms=None):
        """Milliseconds since the last reading, or None if there was none."""
        if self.last_update_ms is None:
            return None
        if now_ms is None:
            now_ms = ticks_ms()
        return ticks_diff(now_ms, self.last_update_ms)


class ToFSensor:
    """One VL53L0X ranging continuously, read without blocking.

    i2c: bus object (see make_i2c). driver_factory(i2c, address) builds the low
    level driver; the default is libs/VL53L0X.py. long_range uses the VCSEL
    periods 18/14 from the department example (longer range, more noise).
    """

    def __init__(self, i2c, name="tof", address=DEFAULT_ADDRESS, offset_mm=0, max_valid_mm=1200,
                 median_size=3, long_range=False, stale_ms=200, require_valid_status=True,
                 driver_factory=None):
        self.i2c = i2c
        self.name = name
        self.address = address
        self.long_range = long_range
        self.stale_ms = stale_ms
        self.processor = RangeProcessor(max_valid_mm, median_size, offset_mm, require_valid_status)
        self._driver_factory = driver_factory
        self._flag = bytearray(1)
        self._result = bytearray(12)
        self.driver = None
        self.available = False
        self.last_error = None
        self.error_count = 0
        self.consecutive_errors = 0

    def begin(self):
        """Initialise the sensor and start continuous ranging. Blocking (~50 ms).

        Returns True on success. On failure the sensor is marked unavailable and
        the reason is kept in last_error; call begin() again to retry.
        """
        try:
            factory = self._driver_factory
            if factory is None:
                from libs.VL53L0X import VL53L0X
                factory = VL53L0X
            driver = factory(self.i2c, self.address)
            if self.long_range:
                driver.set_Vcsel_pulse_period(driver.vcsel_period_type[0], 18)
                driver.set_Vcsel_pulse_period(driver.vcsel_period_type[1], 14)
            driver.start()
        except Exception as e:  # a broken or missing sensor must not stop the robot
            self.driver = None
            self.available = False
            self._record_error(e)
            return False
        self.driver = driver
        self.available = True
        self.consecutive_errors = 0
        self.processor.reset()
        return True

    def poll(self, now_ms=None):
        """Read a new sample if one is ready. Returns True if one was consumed. Never raises."""
        if not self.available:
            return False
        if now_ms is None:
            now_ms = ticks_ms()
        try:
            self.i2c.readfrom_mem_into(self.address, _REG_INTERRUPT_STATUS, self._flag)
            if not self._flag[0] & 0x07:
                self.consecutive_errors = 0
                return False
            self.i2c.readfrom_mem_into(self.address, _REG_RANGE_STATUS, self._result)
            self.i2c.writeto_mem(self.address, _REG_INTERRUPT_CLEAR, b"\x01")
        except OSError as e:
            self._record_error(e)
            return False
        self.consecutive_errors = 0
        status = (self._result[0] >> 3) & 0x0F
        raw_mm = (self._result[10] << 8) | self._result[11]
        self.processor.update(raw_mm, status, now_ms)
        return True

    def _record_error(self, e):
        self.last_error = repr(e)
        self.error_count += 1
        self.consecutive_errors += 1

    def get_distance(self, now_ms=None):
        """Filtered distance in mm, or None (no target / unavailable / stale)."""
        if not self.available:
            return None
        age = self.processor.age_ms(now_ms)
        if age is None or age > self.stale_ms:
            return None
        return self.processor.value

    @property
    def distance_mm(self):
        return self.get_distance()

    @property
    def raw_mm(self):
        return self.processor.last_raw

    @property
    def range_status(self):
        return self.processor.last_status

    def age_ms(self, now_ms=None):
        return self.processor.age_ms(now_ms)

    def is_closer_than(self, mm, now_ms=None):
        d = self.get_distance(now_ms)
        return d is not None and d < mm

    def read_blocking(self, timeout_ms=100):
        """Wait (blocking) for the next sample and return the distance or None. For testing."""
        start = ticks_ms()
        while ticks_diff(ticks_ms(), start) < timeout_ms:
            if self.poll():
                return self.get_distance()
            sleep_ms(1)
        return None

    def stop(self):
        """Stop continuous ranging."""
        if self.driver is not None:
            try:
                self.driver.stop()
            except Exception as e:
                self._record_error(e)
        self.available = False


def setup_tof_sensors(specs, freq=400000, driver_factory=None, **options):
    """Create and start every sensor described in specs (dicts like config.TOF_SENSORS).

    Returns {name: ToFSensor} in spec order. Sensors with an "xshut" pin are held
    in reset first and released one by one, so each can be moved to its
    configured address before the next one wakes up. A sensor that fails stays
    in the dict with available == False. Blocking; call once at start-up.
    """
    xshut = {}
    for spec in specs:
        if spec.get("xshut") is not None:
            xshut[spec["name"]] = Pin(spec["xshut"], Pin.OUT, value=0)
    if xshut:
        sleep_ms(10)

    sensors = {}
    for spec in specs:
        name = spec["name"]
        bus = make_i2c(spec["i2c_id"], spec["sda"], spec["scl"], freq)
        address = spec.get("address", DEFAULT_ADDRESS)
        if name in xshut:
            xshut[name].value(1)
            sleep_ms(10)   # boot time after XSHUT goes high
        sensor = ToFSensor(bus, name=name, address=address, offset_mm=spec.get("offset_mm", 0),
                           driver_factory=driver_factory, **options)
        if address != DEFAULT_ADDRESS and not set_address(bus, DEFAULT_ADDRESS, address):
            sensor.last_error = "could not change address 0x29 -> 0x%02X" % address
            sensor.error_count += 1
        else:
            sensor.begin()
        sensors[name] = sensor
    return sensors


def tof_from_config(driver_factory=None):
    """setup_tof_sensors() with every setting taken from config.py."""
    import config
    return setup_tof_sensors(config.TOF_SENSORS, config.I2C_FREQ_HZ, driver_factory,
                             max_valid_mm=config.TOF_MAX_VALID_MM,
                             median_size=config.TOF_MEDIAN_SIZE,
                             long_range=config.TOF_LONG_RANGE,
                             stale_ms=config.TOF_STALE_MS)
