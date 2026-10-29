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

NODE_FORWARD_MS = 800
TURN_MIN_MS = 800
TURN_SPEED = 80


# NODE_FORWARD_MS = 800
# TURN_MIN_MS = 650
# TURN_SPEED = 100

TURN_TIMEOUT_MS = 3000
LINE_LOST_MS = 100
REVERSE_TIMEOUT_MS = 3300
JUNCTION_HOLDOFF_MS = 270
LONG_PRESS_MS = 1500

DISTANCE_TO_SCAN_PICK_UP = 200
DISTANCE_TO_PICK_UP = 45
SCAN_TURN_TIME = 200



DISTANCE_TO_DROP_OFF_SCAN = 1500
MINUS_DISTANCE_FOR_TOP_DROP_OFF = 700
DISTANCE_TO_DROP_POINT = 1450

DISTANCE_TO_SCAN_DROP_OFF = 370