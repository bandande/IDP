# full_turn_time=20 #constant we change
# # DRIVE_SPEED: percent of full motor power used by Robot.forward / left /
# #     right / backward when no explicit speed is given (motor.Motor's own
# #     default is 100 %).
# # CHANGED (2026-10-20): LINE_TEST_ONLY and the line-test route were removed
# # from main.py (as requested). For line-following tests on the bench use
# # tests/test_line_follow.py.
# DRIVE_SPEED = 60
# # DRIVE_SPEED = 100


# # Turning and junction settings. All times are in MILLISECONDS (`now` in
# # main() counts milliseconds).
# # NODE_FORWARD_MS: the line sensors sit in front of the wheels, so when the
# #     sensors reach a junction the wheels are still short of it. Before
# #     turning on the spot the robot drives on for this long so that the
# #     WHEELS are on the junction. Roughly (distance from the line sensors to
# #     the wheel axle) / (driving speed).
# # TURN_SPEED: motor power (%) of both wheels while turning on the spot.
# # TURN_MIN_MS: a turn never finishes earlier than this.
# # START_TURN_MIN_MS: ADDED (2026-10-20): the same for the first turn at the
# #     start (the start box lines have to be passed). It replaces the
# #     "TURN_MIN_MS=1700" that used to be set inside main(), see the turning
# #     code.
# # JUNCTION_HOLDOFF_MS: CHANGED (2026-10-20): after a junction has been
# #     detected, ignore new junctions for this long, so that the same
# #     junction is not counted twice while the outer sensor slides off its
# #     side line (it can flicker black-white-black). NOT used after a turn:
# #     then the sensors are far from the old junction, and a hold-off could
# #     make the robot miss a junction only a few cm ahead.
# # LONG_PRESS_MS: ADDED (2026-10-20): the start/stop button has two kinds of
# #     stop, see "two kinds of stop" in main(): a short press pauses, a press
# #     held at least this long stops and resets.
# # NODE_FORWARD_MS = 1600
# # TURN_MIN_MS = 1800

# NODE_FORWARD_MS = 1500
# TURN_MIN_MS = 1200
# TURN_SPEED = 60


# # NODE_FORWARD_MS = 800
# # TURN_MIN_MS = 650
# # TURN_SPEED = 100

# TURN_TIMEOUT_MS = 4000
# LINE_LOST_MS = 150
# REVERSE_TIMEOUT_MS = 5000
# JUNCTION_HOLDOFF_MS = 400
# LONG_PRESS_MS = 1500

# DISTANCE_TO_SCAN_PICK_UP = 150
# DISTANCE_TO_PICK_UP = 20
# SCAN_TURN_TIME = 200

# DISTANCE_TO_DROP_OFF_SCAN = 3000
# MINUS_DISTANCE_FOR_TOP_DROP_OFF = 1000
# DISTANCE_TO_DROP_POINT = 3000

# DISTANCE_TO_SCAN_DROP_OFF = 300

full_turn_time=20 #constant we change
# DRIVE_SPEED: percent of full motor power used by Robot.forward / left /
#     right / backward when no explicit speed is given (motor.Motor's own
#     default is 100 %).
# CHANGED (2026-10-20): LINE_TEST_ONLY and the line-test route were removed
# from main.py (as requested). For line-following tests on the bench use
# tests/test_line_follow.py.
DRIVE_SPEED = 90
# DRIVE_SPEED = 100


# Turning and junction settings. All times are in MILLISECONDS (`now` in
# main() counts milliseconds).
# NODE_FORWARD_MS: the line sensors sit in front of the wheels, so when the
#     sensors reach a junction the wheels are still short of it. Before
#     turning on the spot the robot drives on for this long so that the
#     WHEELS are on the junction. Roughly (distance from the line sensors to
#     the wheel axle) / (driving speed).
# TURN_SPEED: motor power (%) of both wheels while turning on the spot.
# TURN_MIN_MS: a turn never finishes earlier than this.
# START_TURN_MIN_MS: ADDED (2026-10-20): the same for the first turn at the
#     start (the start box lines have to be passed). It replaces the
#     "TURN_MIN_MS=1700" that used to be set inside main(), see the turning
#     code.
# JUNCTION_HOLDOFF_MS: CHANGED (2026-10-20): after a junction has been
#     detected, ignore new junctions for this long, so that the same
#     junction is not counted twice while the outer sensor slides off its
#     side line (it can flicker black-white-black). NOT used after a turn:
#     then the sensors are far from the old junction, and a hold-off could
#     make the robot miss a junction only a few cm ahead.
# LONG_PRESS_MS: ADDED (2026-10-20): the start/stop button has two kinds of
#     stop, see "two kinds of stop" in main(): a short press pauses, a press
#     held at least this long stops and resets.
# NODE_FORWARD_MS = 1600
# TURN_MIN_MS = 1800

NODE_FORWARD_MS = 787
TURN_MIN_MS = 900
TURN_SPEED = 70

# CHANGED (2026-10-30, later): 200 -> 100 (the user, after a robot test).
# AFTER_TURN_REVERSE_MS = 100
AFTER_TURN_REVERSE_MS = 000

# ADDED (2026-10-30): the same, but for the turn into a box's line just before
# a pick-up (ms). That line is short, so back up more there. 0 = switched off.
PICKUP_TURN_REVERSE_MS = 300

# PIVOT_TURN_MIN_MS: a pivot turns half as fast as a spin (one wheel instead
#     of two), so twice TURN_MIN_MS before it looks for the new line.
PIVOT_TURN_AT = (("JUNCTION_GREEN_1", "JUNCTION_LEFT_BOTTOM"),
                 ("JUNCTION_RED_1", "JUNCTION_RIGHT_BOTTOM"))
PIVOT_FORWARD_MS = 400
PIVOT_TURN_MIN_MS = 1600


# NODE_FORWARD_MS = 800
# TURN_MIN_MS = 650
# TURN_SPEED = 100

TURN_TIMEOUT_MS = 3000
LINE_LOST_MS = 100
REVERSE_TIMEOUT_MS = 3300
JUNCTION_HOLDOFF_MS = 270
LONG_PRESS_MS = 1500

DISTANCE_TO_SCAN_PICK_UP = 200
DISTANCE_TO_PICK_UP = 58
SCAN_TURN_TIME = 200


# CHANGED (2026-10-30, later): another 
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
# DISTANCE_TO_SCAN_DROP_OFF = 340

CORRECTION_HOLDOFF_MS = 100