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

NODE_FORWARD_MS = 790
TURN_MIN_MS = 800
TURN_SPEED = 80
# ADDED (2026-10-30): after every 90 degree turn (left / right) the robot backs
# up for this long (ms), so the line following that comes next has a little
# more line to straighten the robot on. On the robot the turns before and
# after a pick-up ended slightly past 90 degrees. 0 = switched off.
# CHANGED (2026-10-30, later): 200 -> 100 (the user, after a robot test).
# AFTER_TURN_REVERSE_MS = 100
AFTER_TURN_REVERSE_MS = 000

# ADDED (2026-10-30): the same, but for the turn into a box's line just before
# a pick-up (ms). That line is short, so back up more there. 0 = switched off.
PICKUP_TURN_REVERSE_MS = 300
# ADDED (2026-10-30, experimental): turns about ONE wheel ("pivot", see
# Robot.pivot in functions.py) where the normal turn's drive on pushes the box
# towards the table edge:
#   * at a bottom corner, coming down from the first box's junction
#     (PIVOT_TURN_AT: (previous junction, junction where the robot turns));
#   * at a drop junction, coming out of a slot that already held a box, on
#     the way to the next slot.
# PIVOT_FORWARD_MS: drive on for this long before pivoting (instead of
#     NODE_FORWARD_MS). The inside wheel must stop half the wheel track
#     before the junction centre: the sensors are ~20 cm in front of the
#     axle and the track is ~26 cm (sim/world.py), so drive 20 - 13 = 7 cm
#     instead of 20 cm: 800 * 7 / 20 = 280, about 300 ms.
#     CHANGED (2026-10-30): 300 -> 400 (the user). The real wheel track is
#     not known; tune it with tests/test_turn_calibration.py "pivot_left" /
#     "pivot_right": TOO FAR -> shorter, NOT FAR ENOUGH -> longer.
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
DISTANCE_TO_PICK_UP = 45
SCAN_TURN_TIME = 200



# CHANGED (2026-10-30): drive 0.5 s further into the drop branch before the
# U-turn and the rear sensor check (was 1500), so the rear sensor is closer
# to a box already in the slot. On the robot (2026-10-30 test) 3 of the 4
# full slots were seen as empty from the old position. The top level drives
# MINUS_DISTANCE_FOR_TOP_DROP_OFF less.
# CHANGED (2026-10-30, later): another 0.3 s closer (2000 -> 2300): closer to
# the slot, a box and an empty slot read further apart.
# CHANGED (2026-10-30, later): only 0.15 s of that (2300 -> 2150): at 2300 the
# robot was too close and hit the wall. Top level: 1450.
DISTANCE_TO_DROP_OFF_SCAN = 2100
MINUS_DISTANCE_FOR_TOP_DROP_OFF = 700
# CHANGED (2026-10-30): shorter by exactly as much as DISTANCE_TO_DROP_OFF_SCAN
# grew (1450 -> 950 -> 650), because the robot already stands that much
# further in, so the bottom-level box is put down at the same place as
# before. (The top level drives on until the line ends, it does not use this.)
# CHANGED (2026-10-30, later): 650 -> 560. This drive is now timed from AFTER
# the run-up (DROP_RUNUP_MS); before, the gripper lift (about 90 ms) at its
# start was counted in it. 90 ms less = the same end point as before.
# CHANGED (2026-10-30, later): 560 -> 710, 150 ms longer because
# DISTANCE_TO_DROP_OFF_SCAN became 150 ms shorter: the box is put down where
# the slot check + this drive end, so the end point stays the same.
DISTANCE_TO_DROP_POINT = 710
# ADDED (2026-10-30): before that last drive into the slot the robot backs up
# for this long (ms) and then drives in DISTANCE_TO_DROP_POINT + this, so it
# still ends DISTANCE_TO_DROP_POINT past the slot check. The drive's line
# following stops 800 ms before its end, so it straightens the robot for
# DISTANCE_TO_DROP_POINT + DROP_RUNUP_MS - 800 ms (now 710 ms).
# 0 = switched off.
DROP_RUNUP_MS = 800

# CHANGED (2026-10-30): slot full / empty threshold of the rear sensor (mm),
# was 370. Rear sensor readings at DISTANCE_TO_DROP_OFF_SCAN = 2000 (left
# bottom): full 306, empty 376 - 378. Each 0.1 s closer reads about 16 mm
# less, so at 2300 about full 260, empty 330 -> the middle, 295.
# Measured at 2300 (robot test): full 243 - 252, empty 346.
# CHANGED (2026-10-30, later): 295 -> 320 for DISTANCE_TO_DROP_OFF_SCAN = 2150,
# half way between the two measured places: full about (306 + 252) / 2 = 279,
# empty about (376 + 346) / 2 = 361 -> the middle, 320.
# ESTIMATE: check it against the next test's "rear sensor readings".
DISTANCE_TO_SCAN_DROP_OFF = 325