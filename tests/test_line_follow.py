"""
Step 10: backup line follower on the real track.

Do steps 2 (line sensors), 8 (button + LEDs) and 9 (motors) first.

How to run
  * Put the robot on the line, centred, facing along it.
  * Press the start/stop button (config.START_STOP_SENSOR). The yellow LED
    blinks for START_DELAY_MS so your hand can move away, then it follows.
  * Press the button again (or let the front bump into something) to stop.
  * Ctrl-C always stops the motors.
LEDs: green = following, yellow = turning / on a junction, red = stopped.
A status line is printed every 200 ms, plus junction events and the reason
whenever the follower stops on its own.

Options
  test_line_follow(plan=["L", "S", "R"])  one entry per junction: L, R, S, stop
  test_line_follow(dry_run=True)          motors stay off; push the robot by
                                          hand to watch its decisions
Tuning (config.py): FOLLOW_BASE_SPEED, FOLLOW_KP, FOLLOW_KD,
FOLLOW_MAX_CORRECTION, FOLLOW_SEARCH_*, TURN_*, T_STEM_ACTION, LINE_LOST_MS.
  * Wobbles from side to side: lower FOLLOW_KP or add FOLLOW_KD (e.g. 2-5).
  * Runs off the line in bends: raise FOLLOW_KP or lower FOLLOW_BASE_SPEED.
  * Turns too early / too late at junctions: raise / lower TURN_FORWARD_MS.
"""

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
