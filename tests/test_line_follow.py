from utime import sleep_ms, ticks_ms, ticks_diff, ticks_add

import config
from sensors.line_sensor import LineSensorArray
from sensors.line_processing import bits_to_str
from actuators.motor import DriveBase
from actuators.leds import StatusLeds
from control.start_stop import StartStopButton
from control.line_follower import LineFollower


def test_line_follow(plan=None, dry_run=False, seconds=None, print_ms=200, auto_start=False):
    line = LineSensorArray.from_config()
    follower = LineFollower.from_config()
    button = StartStopButton.from_config()
    leds = StatusLeds.from_config()
    drive = None if dry_run else DriveBase.from_config()
    print("backup line follower%s, plan %s" % (" (DRY RUN, motors off)" if dry_run else "", plan))
    print("press the %s crash sensor to start / stop" % config.START_STOP_SENSOR)
    arm_at = None
    reported_done = False
    last_event = None
    start = ticks_ms()
    last_print = start
    if auto_start:
        follower.start(start, plan)
    try:
        while seconds is None or ticks_diff(ticks_ms(), start) < seconds * 1000:
            now = ticks_ms()
            bits = line.read()

            if button.pressed(now):
                if follower.running or arm_at is not None:
                    follower.stop("button")
                    arm_at = None
                    print(">>> stopped by the button")
                else:
                    arm_at = ticks_add(now, config.START_DELAY_MS)
                    reported_done = False
                    print(">>> starting in %d ms" % config.START_DELAY_MS)
            if arm_at is not None and ticks_diff(now, arm_at) >= 0:
                arm_at = None
                follower.start(now, plan)
                print(">>> following")

            left, right = follower.update(bits, now)
            if drive is not None:
                drive.drive(left, right)

            if follower.last_event is not None and follower.last_event is not last_event:
                last_event = follower.last_event
                print(">>> junction %d: %r" % (follower.junction_count, last_event))
            if follower.state == LineFollower.DONE and not reported_done:
                reported_done = True
                print(">>> stopped: %s" % follower.reason)

            turning = follower.state in (LineFollower.TURN_FORWARD, LineFollower.TURN_SPIN)
            if arm_at is not None:
                leds.show(yellow=(now // 125) % 2 == 0, green=False, red=True)
            else:
                leds.show(green=follower.running, red=not follower.running,
                          yellow=turning or follower.junctions.in_junction)

            if ticks_diff(now, last_print) >= print_ms:
                last_print = now
                print("%-12s %s err %+.2f  L %4d R %4d  junctions %d %s" % (
                    follower.state, bits_to_str(bits), follower.error, follower.left,
                    follower.right, follower.junction_count,
                    ("turn " + follower.turn_dir) if turning else ""))
            sleep_ms(config.LOOP_PERIOD_MS)
    finally:
        if drive is not None:
            drive.stop()
        leds.off()
        button.deinit()
        print("motors stopped")


if __name__ == "__main__":
    test_line_follow()
