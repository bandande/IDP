# DRIVE_SPEED: percent of full motor power used by Robot.forward / left /
#     right / backward when no explicit speed is given (motor.Motor's own
#     default is 100 %).
DRIVE_SPEED = 90


# Turning and junction settings. All times are in MILLISECONDS (`now` in
# main() counts milliseconds).
# NODE_FORWARD_MS: the line sensors sit in front of the wheels, so when the
#     sensors reach a junction the wheels are still short of it. Before
#     turning on the spot the robot drives on for this long so that the
#     WHEELS are on the junction. Roughly (distance from the line sensors to
#     the wheel axle) / (driving speed).
# TURN_SPEED: motor power (%) of both wheels while turning on the spot.
# TURN_MIN_MS: a turn never finishes earlier than this.

NODE_FORWARD_MS = 787
TURN_MIN_MS = 900
TURN_SPEED = 70

AFTER_TURN_REVERSE_MS = 000

PICKUP_TURN_REVERSE_MS = 300

# PIVOT_TURN_MIN_MS: a pivot turns half as fast as a spin (one wheel instead
#     of two), so twice TURN_MIN_MS before it looks for the new line.
PIVOT_TURN_AT = (("JUNCTION_GREEN_1", "JUNCTION_LEFT_BOTTOM"),
                 ("JUNCTION_RED_1", "JUNCTION_RIGHT_BOTTOM"))
PIVOT_FORWARD_MS = 400
PIVOT_TURN_MIN_MS = 1600



TURN_TIMEOUT_MS = 3000
LINE_LOST_MS = 100
JUNCTION_HOLDOFF_MS = 270
LONG_PRESS_MS = 1500

DISTANCE_TO_PICK_UP = 58


DISTANCE_TO_DROP_OFF_SCAN = 1500
MINUS_DISTANCE_FOR_TOP_DROP_OFF = 620
MINUS_DISTANCE_TO_DROP_OFF_SCAN_RIGHT = 290
# DISTANCE_TO_DROP_OFF_SCAN became 150 ms shorter: the box is put down where
# the slot check + this drive end, so the end point stays the same.
DISTANCE_TO_DROP_POINT = 780
# DISTANCE_TO_DROP_POINT + DROP_RUNUP_MS - 800 ms (now 710 ms).
# 0 = switched off.
DROP_RUNUP_MS = 800

DISTANCE_TO_SCAN_DROP_OFF = 325

CORRECTION_HOLDOFF_MS = 100