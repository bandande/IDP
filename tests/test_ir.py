from utime import sleep_ms, ticks_ms, ticks_diff

from sensors.ir_receiver import IRReceiver, decode_nec, frame_signature


def test_ir(seconds=None):
    ir = IRReceiver.from_config()
    present = False
    print("IR absent")
    start = ticks_ms()
    while seconds is None or ticks_diff(ticks_ms(), start) < seconds * 1000:
        ir.process()
        now_present = ir.signal_present()
        if now_present != present:
            present = now_present
            print("IR PRESENT" if present else "IR absent")
        frame = ir.get_frame()
        while frame is not None:
            n, mark, space, total = frame_signature(frame)
            print("frame: %d marks, mean mark %d us, mean space %d us, %d us total" % (
                n, mark, space, total))
            print("  decode_nec:", decode_nec(frame))
            print("  template:", frame)
            frame = ir.get_frame()
        sleep_ms(5)
    if ir.overflow_count or ir.glitch_count:
        print("overflow %d, glitches %d" % (ir.overflow_count, ir.glitch_count))
    ir.deinit()


if __name__ == "__main__":
    test_ir()
