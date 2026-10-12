"""
Small signal filters shared by the sensor modules.

Everything here is pure Python with no hardware access, so it can be unit
tested on a PC (see tests/host). Time-based classes take an optional `now_ms`
argument; when it is omitted the current `utime.ticks_ms()` is used.
"""

from utime import ticks_ms, ticks_diff


class MedianFilter:
    """Median of the last `size` samples.

    Removes single-sample spikes (typical for time-of-flight sensors). Use an
    odd size such as 3 or 5. While fewer than `size` samples have arrived the
    lower median of the samples so far is returned.
    """

    def __init__(self, size=3):
        if size < 1:
            raise ValueError("size must be >= 1")
        self.size = size
        self._buf = [0] * size
        self._count = 0
        self._index = 0
        self.value = None

    def reset(self):
        self._count = 0
        self._index = 0
        self.value = None

    def update(self, x):
        self._buf[self._index] = x
        self._index = (self._index + 1) % self.size
        if self._count < self.size:
            self._count += 1
        window = sorted(self._buf[:self._count])
        self.value = window[(self._count - 1) // 2]
        return self.value


class Debouncer:
    """Time-based debouncer for a boolean signal.

    The output `state` only changes after the raw input has held the new value
    for at least `stable_ms`. After each update(), `rose` / `fell` tell whether
    the debounced output changed on that call.
    """

    def __init__(self, stable_ms, initial=False):
        self.stable_ms = stable_ms
        self.state = bool(initial)
        self.rose = False
        self.fell = False
        self._since = None

    def reset(self, state=False):
        self.state = bool(state)
        self.rose = False
        self.fell = False
        self._since = None

    def update(self, raw, now_ms=None):
        if now_ms is None:
            now_ms = ticks_ms()
        raw = bool(raw)
        self.rose = False
        self.fell = False
        if raw == self.state:
            self._since = None
            return self.state
        if self._since is None:
            self._since = now_ms
        if ticks_diff(now_ms, self._since) >= self.stable_ms:
            self.state = raw
            self._since = None
            if raw:
                self.rose = True
            else:
                self.fell = True
        return self.state


class Hysteresis:
    """Two-threshold switch, e.g. "object is near" for a distance signal.

    `state` becomes True when value <= on_below, False when value >= off_above,
    and keeps its previous value in between. A `None` value (no reading) sets
    the state to `none_state`; pass none_state=None to keep the previous state.
    """

    def __init__(self, on_below, off_above, initial=False, none_state=False):
        if on_below > off_above:
            raise ValueError("on_below must be <= off_above")
        self.on_below = on_below
        self.off_above = off_above
        self.none_state = none_state
        self.state = bool(initial)

    def reset(self, state=False):
        self.state = bool(state)

    def update(self, value):
        if value is None:
            if self.none_state is not None:
                self.state = bool(self.none_state)
        elif value <= self.on_below:
            self.state = True
        elif value >= self.off_above:
            self.state = False
        return self.state
