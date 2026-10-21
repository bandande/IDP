"""Fake `micropython` module (only what the robot code uses)."""


def const(x):
    return x


def alloc_emergency_exception_buf(size):
    pass


def schedule(func, arg):
    func(arg)
    return True


def opt_level(level=None):
    return 0


def mem_info(*args):
    pass
