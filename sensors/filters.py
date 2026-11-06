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

