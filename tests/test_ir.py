"""
Step 4: TSOP4838 IR receiver.

Wiring (lens facing you, legs down, left to right): 1 OUT -> GP14
(physical 19, moved from GP15 in the schematic on 2026-10-14), 2 GND -> GND
(physical 38), 3 VS -> 3V3_OUT (physical 36, NOT 5V). No extra parts needed.
Optional if the reading is noisy near the motors: about 100 ohm in series with
VS and a few uF from VS to GND, close to the sensor (datasheet R1/C1).

Checks:
  1. At rest: "absent", no frames.
  2. Point any TV remote at it and press a button: "PRESENT", frames printed.
     Most remotes use NEC -> a decoded address/command is shown.
  3. Point the traffic light at it: copy the printed frame list into the code
     as a template and recognise it later with match_template().
Stop with Ctrl-C.
"""

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
