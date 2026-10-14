"""
Interface for the Vishay TSOP4838 IR receiver module.

What the device does
  * It contains a photodiode, an amplifier, a 38 kHz band-pass filter and a
    demodulator. It does NOT pass the 38 kHz carrier through: its OUT pin goes
    LOW while it receives a burst of 38 kHz modulated IR and is HIGH otherwise.
    Software therefore sees "marks" (LOW = IR burst present) and "spaces"
    (HIGH = no IR), and information is carried by their durations.
  * Pins (looking at the lens): 1 = OUT, 2 = GND, 3 = VS. Supply 2.0-5.5 V.
    OUT has an internal 30k pull-up to VS, so power VS from 3V3: at 5V the
    output would put 5V on the Pico pin. If the supply is noisy (motors), add
    the RC filter from the datasheet (series resistor into VS, capacitor to GND).
  * It is an AGC2 part made for remote-control data: bursts must be at least
    10 carrier cycles (about 0.26 ms) and be followed by gaps. A CONTINUOUS
    38 kHz signal (like DC light or lamps) is treated as noise and suppressed,
    so a beacon that is simply "always on" may only give one short LOW pulse.
    Beacons that send bursts or remote-control codes work well.

What this module does
  * A hard IRQ timestamps every edge into a pre-allocated ring buffer.
  * process() (call it often from the main loop) turns edges into frames:
    lists of durations in microseconds, alternating mark, space, mark, ...,
    always starting and ending with a mark. A frame ends after frame_gap_us
    of silence.
  * signal_present(): IR activity within the last present_window_ms.
  * decode_nec(frame): decodes the NEC protocol (the most common remote
    control format; addr, ~addr, cmd, ~cmd, LSB first).
  * frame_signature() / match_template(): for unknown beacons. Print a frame
    once (print(ir.get_frame())), paste it into the code as a template, then
    recognise it later with match_template(frame, template).

Usage
    from sensors.ir_receiver import IRReceiver, decode_nec
    ir = IRReceiver.from_config()
    while True:
        ir.process()
        if ir.signal_present():
            ...
        frame = ir.get_frame()           # list of durations in us, or None
        if frame is not None:
            code = decode_nec(frame)     # IRCode, or None if it is not NEC
"""

import array
import machine
import micropython
from machine import Pin
from utime import ticks_us, ticks_diff

micropython.alloc_emergency_exception_buf(100)

# NEC protocol timings in microseconds.
NEC_HEADER_MARK = 9000
NEC_HEADER_SPACE = 4500
NEC_REPEAT_SPACE = 2250
NEC_BIT_MARK = 560
NEC_ZERO_SPACE = 560
NEC_ONE_SPACE = 1690
_MIN_MARGIN_US = 200   # the receiver stretches / shrinks pulses by up to ~100-200 us


def _near(value, expected, tolerance, min_abs_us=_MIN_MARGIN_US):
    margin = expected * tolerance
    if margin < min_abs_us:
        margin = min_abs_us
    return abs(value - expected) <= margin


class IRCode:
    """A decoded remote-control code. address is None for a repeat code."""

    def __init__(self, protocol, address, command, repeat=False):
        self.protocol = protocol
        self.address = address
        self.command = command
        self.repeat = repeat

    def __repr__(self):
        if self.repeat:
            return "IRCode(%s repeat)" % self.protocol
        return "IRCode(%s, address=0x%02X, command=0x%02X)" % (self.protocol, self.address, self.command)


def decode_nec(durations, tolerance=0.3):
    """Decode an NEC frame (list of us durations starting with a mark).

    Returns IRCode, IRCode with repeat=True for the NEC repeat frame, or None.
    A 16-bit "extended NEC" address is returned when the second byte is not
    the inverse of the first.
    """
    if not durations:
        return None
    n = len(durations)
    if not _near(durations[0], NEC_HEADER_MARK, tolerance):
        return None
    if n == 3 and _near(durations[1], NEC_REPEAT_SPACE, tolerance) \
            and _near(durations[2], NEC_BIT_MARK, tolerance):
        return IRCode("NEC", None, None, True)
    if n != 67 or not _near(durations[1], NEC_HEADER_SPACE, tolerance):
        return None
    data = [0, 0, 0, 0]
    for i in range(32):
        mark = durations[2 + 2 * i]
        space = durations[3 + 2 * i]
        if not _near(mark, NEC_BIT_MARK, tolerance):
            return None
        if _near(space, NEC_ONE_SPACE, tolerance):
            data[i >> 3] |= 1 << (i & 7)
        elif not _near(space, NEC_ZERO_SPACE, tolerance):
            return None
    if not _near(durations[66], NEC_BIT_MARK, tolerance):
        return None
    address, address_inv, command, command_inv = data
    if command ^ command_inv != 0xFF:
        return None
    if address ^ address_inv != 0xFF:
        address = address | (address_inv << 8)
    return IRCode("NEC", address, command, False)


def frame_signature(durations):
    """(n_marks, mean_mark_us, mean_space_us, total_us) - a quick summary of any frame."""
    if not durations:
        return (0, 0, 0, 0)
    marks = durations[0::2]
    spaces = durations[1::2]
    mean_mark = sum(marks) // len(marks)
    mean_space = sum(spaces) // len(spaces) if spaces else 0
    return (len(marks), mean_mark, mean_space, sum(durations))


def match_template(durations, template, tolerance=0.25, min_abs_us=150):
    """True if a frame has the same length as a recorded template and every
    duration is within max(tolerance * expected, min_abs_us) of it."""
    if not durations or len(durations) != len(template):
        return False
    for value, expected in zip(durations, template):
        if not _near(value, expected, tolerance, min_abs_us):
            return False
    return True


class IRReceiver:
    """TSOP4838 on one GPIO: edge capture, frame assembly and presence detection.

    buf_size: edge ring buffer size (power of two). max_frames: completed frames
    kept until read with get_frame() (oldest dropped first).
    """

    def __init__(self, pin, frame_gap_us=8000, buf_size=256, max_frames=8, present_window_ms=150):
        if buf_size < 8 or buf_size & (buf_size - 1):
            raise ValueError("buf_size must be a power of two >= 8")
        if max_frames < 1:
            raise ValueError("max_frames must be >= 1")
        self.frame_gap_us = frame_gap_us
        self.present_window_us = present_window_ms * 1000
        self.max_frames = max_frames
        self._mask = buf_size - 1
        self._times = array.array("L", [0] * buf_size)
        self._levels = bytearray(buf_size)
        self._head = 0           # next slot the IRQ writes (only the IRQ changes it)
        self._tail = 0           # next slot process() reads (only process() changes it)
        self.overflow_count = 0  # edges lost because the buffer was full
        self.edge_count = 0
        self.glitch_count = 0    # frames dropped because an edge was missed
        self.frames_dropped = 0
        self._frames = []
        self._frame = None       # durations of the frame being assembled
        self._have_edge = False
        self._recent = False
        self._last_edge_us = 0
        self._last_level = 1
        self._pin = Pin(pin, Pin.IN, Pin.PULL_UP)
        self._irq = self._pin.irq(handler=self._isr, trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING,
                                  hard=True)

    @classmethod
    def from_config(cls):
        import config
        return cls(config.IR_RX_PIN, frame_gap_us=config.IR_FRAME_GAP_US,
                   present_window_ms=config.IR_PRESENT_WINDOW_MS)

    def _isr(self, pin):
        # Hard IRQ: writes only into pre-allocated buffers, no allocation.
        head = self._head
        nxt = (head + 1) & self._mask
        if nxt == self._tail:
            self.overflow_count += 1
            return
        self._times[head] = ticks_us()
        self._levels[head] = pin.value()
        self._head = nxt

    def process(self, now_us=None):
        """Consume captured edges and complete frames. Non-blocking; call every loop."""
        irq_state = machine.disable_irq()
        head = self._head
        machine.enable_irq(irq_state)
        tail = self._tail
        while tail != head:
            self._edge(self._times[tail], self._levels[tail])
            tail = (tail + 1) & self._mask
            self._tail = tail
        if not self._have_edge:
            return
        if now_us is None:
            now_us = ticks_us()
        quiet_us = ticks_diff(now_us, self._last_edge_us)
        if quiet_us > self.present_window_us:
            self._recent = False
        if self._frame is not None and self._last_level == 1 and quiet_us > self.frame_gap_us:
            self._finish_frame()

    def _edge(self, t, level):
        self.edge_count += 1
        self._recent = True
        if not self._have_edge:
            self._have_edge = True
            self._last_edge_us = t
            self._last_level = level
            if level == 0:
                self._frame = []
            return
        dt = ticks_diff(t, self._last_edge_us)
        prev = self._last_level
        self._last_edge_us = t
        self._last_level = level
        if level == prev:
            # Two edges with the same level: an edge was missed, so the
            # durations are unreliable. Drop the frame and resync.
            self.glitch_count += 1
            self._frame = [] if level == 0 else None
            return
        if level == 0:
            # Mark starts; dt was a space.
            if self._frame is not None and dt > self.frame_gap_us:
                self._finish_frame()
            if self._frame is None:
                self._frame = []
            else:
                self._frame.append(dt)
        elif self._frame is not None:
            # Mark ends; dt was a mark.
            self._frame.append(dt)

    def _finish_frame(self):
        frame = self._frame
        self._frame = None
        if not frame:
            return
        if len(self._frames) >= self.max_frames:
            self._frames.pop(0)
            self.frames_dropped += 1
        self._frames.append(frame)

    def signal_present(self, now_us=None):
        """True if the output is LOW now or there was IR activity within present_window_ms."""
        if now_us is None:
            now_us = ticks_us()
        self.process(now_us)
        if self._pin.value() == 0:
            return True
        return self._recent and ticks_diff(now_us, self._last_edge_us) <= self.present_window_us

    def raw(self):
        """Current output level: 0 = IR burst being received, 1 = idle."""
        return self._pin.value()

    def any(self):
        """Number of complete frames waiting."""
        return len(self._frames)

    def get_frame(self):
        """Oldest complete frame (list of us durations, mark first), or None."""
        if self._frames:
            return self._frames.pop(0)
        return None

    def clear(self):
        """Drop queued frames and any frame in progress."""
        self._frames = []
        self._frame = None

    def deinit(self):
        """Disable the IRQ."""
        self._pin.irq(handler=None)
