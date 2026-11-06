from sim.fakes import _bus

_PERIOD = 1 << 30
_MASK = _PERIOD - 1
_HALF = _PERIOD >> 1

# Simulated time spent by one call of a time function (microseconds).
CALL_COST_US = 20

_own_us = [0]   # used only when no world is registered


def _advance(us):
    if _bus.world is not None:
        _bus.world.advance(us)
    else:
        _own_us[0] += us


def _now_us():
    if _bus.world is not None:
        return _bus.world.now_us
    return _own_us[0]


def ticks_us():
    _advance(CALL_COST_US)
    return _now_us() & _MASK


def ticks_ms():
    _advance(CALL_COST_US)
    return (_now_us() // 1000) & _MASK



def ticks_diff(a, b):
    return ((a - b + _HALF) & _MASK) - _HALF


def ticks_add(t, delta):
    return (t + delta) & _MASK


def sleep_us(us):
    _advance(max(0, int(us)))


def sleep_ms(ms):
    _advance(max(0, int(ms)) * 1000)


def sleep(s):
    _advance(max(0, int(s * 1000000)))


def time():
    return _now_us() // 1000000

