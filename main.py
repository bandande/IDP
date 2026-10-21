import functions
import MAP
import utime
import motor
import sensors.line_sensor as line_sensor
# ADDED (2026-10-15): needed by the fixes / bench-test changes below.
#   - the bit masks name each line sensor inside LineSensorArray.read()
#   - StartStopButton uses a crash sensor as the start / stop button
#   - machine is used to switch the motors off when the program ends
#   - config holds START_STOP_SENSOR and START_DELAY_MS
import machine
import config
import re
from sensors.line_processing import OUTER_LEFT, INNER_LEFT, INNER_RIGHT, OUTER_RIGHT
from control.start_stop import StartStopButton
# ADDED (2026-10-20): the three task sensors (control/task_sensors.py):
#   - magnet sensor: north / south pole of the box at pick-up -> rack level
#   - front distance sensor: is the drop slot already full?
#   - IR receiver: traffic light at the ramp entrance (IR = green)
# Their settings (marked TODO where still unknown) are in config.py under
# "Task sensors". A sensor that fails to start is left out with a warning
# and its check uses a safe fallback.
from control.task_sensors import TaskSensors

# DRIVE_SPEED: percent of full motor power used by Robot.forward / left /
#     right / backward when no explicit speed is given (motor.Motor's own
#     default is 100 %).
# CHANGED (2026-10-20): LINE_TEST_ONLY and the line-test route were removed
# from main.py (as requested). For line-following tests on the bench use
# tests/test_line_follow.py.
DRIVE_SPEED = 60
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

NODE_FORWARD_MS = 1600
TURN_MIN_MS = 1200
TURN_TIMEOUT_MS = 4000
LINE_LOST_MS = 150
REVERSE_TIMEOUT_MS = 5000
JUNCTION_HOLDOFF_MS = 400
LONG_PRESS_MS = 1500

# ADDED (2026-10-20): neighbouring nodes that are CLOSER to each other than
# the line sensors are in front of the wheels (SENSOR_AHEAD_M). After the
# robot TURNS on the spot at the first node, its sensors are already past the
# second one and never see it, so the next junction would be counted as the
# second node and every junction after it would be one off. For these pairs
# the second node is counted as reached straight after the turn (see
# "already behind the line sensors" in main()). Driving STRAIGHT through the
# first node is not affected: then the sensors do see the second node.
# The number is the distance between the two nodes (metres): the wheels are
# then that far short of the second node, so if the robot turns there, it
# first drives only that far (NODE_FORWARD_MS * distance / SENSOR_AHEAD_M)
# instead of the full NODE_FORWARD_MS.
#   - START and the start box edges: ~8-15 cm
#   - the two pad junctions on each side line: ~17 cm in the video
#   - neighbouring drop junctions on the same line (one rack slot apart):
#     RUNG_SPACING_M, checked in close_after_turn() below
# TODO: measure these on the table (and SENSOR_AHEAD_M on the robot). A pair
# that is more than SENSOR_AHEAD_M apart must be removed.
SENSOR_AHEAD_M = 0.20
RUNG_SPACING_M = 0.12
CLOSE_AFTER_TURN = (
    ("START", "MIDDLE_LEFT", 0.15), ("START", "MIDDLE_RIGHT", 0.15), ("START", "TOP_CENTRE", 0.08),
    ("JUNCTION_GREEN_1", "JUNCTION_GREEN_2", 0.17), ("JUNCTION_GREEN_2", "JUNCTION_GREEN_1", 0.17),
    ("JUNCTION_RED_1", "JUNCTION_RED_2", 0.17), ("JUNCTION_RED_2", "JUNCTION_RED_1", 0.17),
)


def close_after_turn(a, b):
    """ADDED (2026-10-20): if node b is so close to node a that, after the
    robot turns on the spot at a, its line sensors are already past b,
    return the distance between them (metres); otherwise None. See
    CLOSE_AFTER_TURN above. Neighbouring drop junctions on the same line,
    e.g. JUNCTION_LEFT_BOTTOM_DROP_2 and _3, are RUNG_SPACING_M apart."""
    for first, second, distance in CLOSE_AFTER_TURN:
        if (first, second) == (a, b):
            return distance
    if a.startswith("JUNCTION_") and b.startswith("JUNCTION_") and "_DROP_" in a and "_DROP_" in b:
        same_line = a[:-1] == b[:-1]                   # same name apart from the slot number
        if same_line and abs(int(a[-1]) - int(b[-1])) == 1:
            return RUNG_SPACING_M
    return None


def corrections(robot, motor3, motor4, motor_speed3=TURN_SPEED, motor_speed4=TURN_SPEED, reverse=None):
    if reverse:
        if robot.front_left_sensor == 0 and robot.front_right_sensor == 1:
            # If left front sensor does not detect and right does
            robot.queue.append(robot.mode)
            
            robot.mode="correction"
            robot.right(motor3,motor4,motor_speed3, motor_speed4, reverse=reverse)
            #Then turn right
            
        elif robot.front_left_sensor == 1 and robot.front_right_sensor == 0:
            # If left front sensor does not detect and right does
            robot.queue.append(robot.mode)
            robot.mode="correction"
            robot.left(motor3,motor4,motor_speed3, motor_speed4, reverse=reverse)
            #Then turn left
    else:
        if robot.front_left_sensor == 0 and robot.front_right_sensor == 1:
            # If left front sensor does not detect and right does
            robot.queue.append(robot.mode)
            robot.mode="correction"
            robot.right(motor3,motor4,motor_speed3, motor_speed4)
            #Then turn right
            
        elif robot.front_left_sensor == 1 and robot.front_right_sensor == 0:
            # If left front sensor does not detect and right does
            robot.queue.append(robot.mode)
            robot.mode="correction"
            robot.left(motor3,motor4,motor_speed3, motor_speed4)
            #Then turn left

def update_sensors(sensors, robot):
    robot.left_sensor_previous=robot.left_sensor
    robot.front_left_sensor_previous=robot.front_left_sensor
    robot.front_right_sensor_previous=robot.front_right_sensor
    robot.right_sensor_previous=robot.right_sensor
    
    bits = sensors.read()
    robot.left_sensor = 1 if bits & OUTER_LEFT else 0
    robot.front_left_sensor = 1 if bits & INNER_LEFT else 0
    robot.front_right_sensor = 1 if bits & INNER_RIGHT else 0
    robot.right_sensor = 1 if bits & OUTER_RIGHT else 0


def wait_for_start(button):
    """ADDED (2026-10-15, safety): block until the start/stop button is pressed.

    The button is the crash sensor named in config.START_STOP_SENSOR (default
    "left" = Crash Sensor L, physical pin 12). Because the Pico runs main.py
    by itself at power-up, waiting here stops the robot from driving off as
    soon as it is switched on. After the press it waits config.START_DELAY_MS
    so the hand that pressed the button is clear of the robot.
    """
    print("press the start/stop button (%s crash sensor) to start" % config.START_STOP_SENSOR)
    while not button.pressed():
        utime.sleep_ms(5)
        # print("hello")
    print("starting in %d ms" % config.START_DELAY_MS)
    utime.sleep_ms(config.START_DELAY_MS)





def main():
    # Status booleans
    turning=False   # Checks if the turning sequence is running so we just start it once
    traffic=False   # Is there traffic? If so, stop all
    moving=False 
    reversing=False # Is the robot reversing or not?
    time_count=0    # How we keep track of a deadline timewise. Changed by saying time_count=now+3
    using_time=False    #Are we actually using the time right now or not?
    is_started=False    # Have we started
    count=0
    at_start=True
    
    
    # ADDED (2026-10-15): state for the fixed turns.
    left_old_line=False   # turning: True once both centre sensors have left the old line
    holdoff_until=0       # ignore junctions until now reaches this (see JUNCTION_HOLDOFF_MS)
    # ADDED (2026-10-20): True right after the robot has turned on the spot at a
    # junction to look into a full slot. Its wheels are then already on the
    # junction, so the next turn must not drive NODE_FORWARD_MS forwards first
    # (that would drive it into the rack).
    skip_forward=False
    # ADDED (2026-10-20): set when a node was counted as reached right after a
    # turn (see CLOSE_AFTER_TURN): the wheels are still short of it, so the
    # NEXT turn only drives this long first (ms). None = the normal
    # NODE_FORWARD_MS. after_turn_forward_ms holds the value until the
    # current turn has finished.
    next_forward_ms=None
    after_turn_forward_ms=None

    
    robot=functions.Robot()
    print(robot.position)
    functions.reset_map()
    map=MAP.MAP.copy()
    # Measure elapsed time
    start = utime.ticks_ms()
    # elapsed = utime.ticks_diff(utime.ticks_ms(), start)
    
    motor3 = motor.Motor(dirPin=4, PWMPin=5)  # Motor 3 is controlled from Motor Driv2 #1, which is on GP4/5
    motor4 = motor.Motor(dirPin=7, PWMPin=6)  # Motor 4 is controlled from Motor Driv2 #1, which is on GP4/5
    # ADDED (2026-10-15): use DRIVE_SPEED (top of this file) instead of the
    # Motor class default of 100 %. Robot.forward / left / right / backward /
    # about use these two values whenever no explicit speed is passed.
    motor3.forward_speed = DRIVE_SPEED
    motor3.backward_speed = DRIVE_SPEED
    motor4.forward_speed = DRIVE_SPEED
    motor4.backward_speed = DRIVE_SPEED

    sensors=line_sensor.LineSensorArray([20,21,19,18],True)

    # ADDED (2026-10-20): start the task sensors. calibrate() measures the
    # magnet sensor's no-magnet zero point, so it runs here, in the start box,
    # before the start button is pressed (no box / magnet near the robot).
    tasks = TaskSensors.from_config()
    print("magnet sensor zero point: %s mV" % tasks.calibrate())

    # ADDED (2026-10-15, safety): start / stop button. Nothing moves until
    # the button is pressed (see wait_for_start above).
    button = StartStopButton.from_config()
    wait_for_start(button)
    is_started=True
    #start running
    while True:
        # FIX (2026-10-20): read the button ONCE per loop. button.pressed()
        # returns True only once for each press. The old code called it twice:
        #     if button.pressed() and not is_started:    (1)
        #     ...
        #         if button.pressed():                   (2)
        # While the robot was running, call (1) used up the press and then
        # threw it away (because is_started was True), so call (2) almost
        # never saw it: that is why the button sometimes did not stop the
        # robot.
        pressed = button.pressed()
        if pressed and not is_started:
            is_started=True
            print("Started")
            pressed = False          # this press only started the robot

        if is_started:
            # CHANGED (2026-10-20): two kinds of stop (the button is the left
            # front crash sensor, so bumping into something also stops):
            #   * SHORT press = PAUSE: the motors stop and the robot keeps
            #     everything (position, queue, mode, timers). The next press
            #     continues exactly where it stopped.
            #   * LONG press (held for LONG_PRESS_MS) = STOP AND RESET: the
            #     robot forgets its route and position (robot.reset(): START,
            #     facing north). Put it back in the start box, then press to
            #     start again. What it has collected / delivered so far stays
            #     marked in the map.
            # (The old 2 x utime.sleep(1) after a stop were removed: a press
            # during them was remembered and restarted the robot at once.)
            if pressed:
                # Remember what both motors were doing, so a pause can resume.
                saved_motors = [(m.mDir.value(), m.pwm.duty_u16()) for m in (motor3, motor4)]
                robot.stop(motor3, motor4)
                pause_start = utime.ticks_ms()
                long_press = False
                while button.held():
                    button.pressed()          # keeps the button's debouncer up to date
                    if utime.ticks_diff(utime.ticks_ms(), pause_start) >= LONG_PRESS_MS:
                        long_press = True
                        break
                    utime.sleep_ms(10)
                if long_press:
                    print("STOP AND RESET: put the robot back in the start box, then press to start")
                    while button.held():      # wait until the button is let go
                        button.pressed()
                        utime.sleep_ms(10)
                    robot.reset()
                    turning=False
                    reversing=False
                    skip_forward=False
                    next_forward_ms=None
                    after_turn_forward_ms=None
                    holdoff_until=0
                    at_start=True
                    wait_for_start(button)
                    start = utime.ticks_ms()
                else:
                    print("PAUSED: press the button again to continue from here")
                    while not button.pressed():
                        utime.sleep_ms(5)
                    utime.sleep_ms(config.START_DELAY_MS)   # hand clear of the robot
                    # Shift the time reference by the length of the pause, so
                    # `now` does not jump and every running timer (time_count)
                    # still has the time it had left before the pause.
                    paused_ms = utime.ticks_diff(utime.ticks_ms(), pause_start)
                    start = utime.ticks_add(start, paused_ms)
                    for m, (direction, duty) in zip((motor3, motor4), saved_motors):
                        m.mDir.value(direction)
                        m.pwm.duty_u16(duty)
                    print("continuing after a %d ms pause" % paused_ms)
                continue

            count+=1
            now = utime.ticks_diff(utime.ticks_ms(), start)
            update_sensors(sensors, robot)

            if len(robot.queue)==0:
                robot.box_count+=1
                robot.queue=["go_back","deliver","collect"]
            # Action
            if robot.mode == "":
                next_item=robot.queue.pop()
            
                if isinstance(next_item, list):
                    # If turning etc
                    robot.mode=next_item[0]
                    robot.mode_specific=next_item[1:]
                    # functions.degrees_to_time(next_item[2]) # gets the count for that specific turn
                    # Write some code to get or use that time
                
                elif next_item.isupper():
                    print("destination:"+next_item)
                    print("current:"+robot.position)

                    robot.queue+=robot.move_to(next_item)
                    
                    # If needs to reverse (dead_end), then if coming from drop off and going to
                    if ((next_item.startswith("JUNCTION_GREEN_") or next_item.startswith("JUNCTION_RED_")) 
                        and (robot.position.startswith("GREEN_") or robot.position.startswith("RED_"))):
                        robot.queue = robot.queue[:-2]+["reverse_straight"]
                        print(robot.queue)
                    
                    # Prevent moving forwards when picking up the box.
                    if ((next_item.startswith("GREEN_") or next_item.startswith("RED_")) 
                        and (robot.position.startswith("JUNCTION_GREEN_") or robot.position.startswith("JUNCTION_RED_"))):
                        # Remove straight
                        robot.queue = robot.queue[:-2]+[robot.queue[-1]]
                        print(robot.queue)
                    
                    # Prevent moving forwards when picking up the box.
                    if ((bool(re.match(r"^JUNCTION.*DROP",next_item))) 
                        and bool(re.match(r"^(?!.*JUNCTION).*DROP",robot.position))):
                        # Remove straight
                        robot.queue = robot.queue[:-2]+[robot.queue[-1]]
                        print(robot.queue)

                    # ADDED (2026-10-20): next_item already behind the line
                    # sensors (see CLOSE_AFTER_TURN at the top). If the robot is
                    # about to TURN here and next_item is closer than the
                    # sensors are in front of the wheels, the sensors will be
                    # past it after the turn and never see it. Count it as
                    # reached at once: drop the "straight" (the drive to it)
                    # and move the position on; the turn itself still happens.
                    # Example: START -> MIDDLE_LEFT, where the start box edge is
                    # ~15 cm from START (this replaces the START.west "map
                    # bodge" in MAP.py, which only worked for the first move).
                    here = robot.position
                    gap = close_after_turn(here, next_item)
                    if (gap is not None and robot.queue
                            and isinstance(robot.queue[-1], list) and robot.queue[-1][1] != "straight"):
                        robot.queue = robot.queue[:-2] + [robot.queue[-1]]
                        robot.position = next_item
                        after_turn_forward_ms = int(NODE_FORWARD_MS * gap / SENSOR_AHEAD_M)
                        print("%s is already behind the line sensors after the turn" % next_item)

                    # ADDED (2026-10-20): traffic light. If the stretch from
                    # here to next_item is in config.TRAFFIC_LIGHT_EDGES (the
                    # ramp entrance: START -> TOP_CENTRE), put "wait_light"
                    # between the turn and the drive. move_to() ended the queue
                    # with [..., "straight", turn] and the queue is used from
                    # the END, so this becomes: turn -> wait_light -> straight.
                    if (here, next_item) in config.TRAFFIC_LIGHT_EDGES:
                        robot.queue = robot.queue[:-1] + ["wait_light", robot.queue[-1]]

                    # ADDED (2026-10-20): look into a drop slot before driving
                    # in. When next_item is a drop slot (e.g. LEFT_BOTTOM_DROP_3),
                    # put "check_slot" between the turn into the slot and the
                    # drive in: turn -> check_slot -> straight -> drop_off.
                    # After the turn the robot is at the junction, facing the
                    # slot, far enough away for the distance sensor to measure.
                    if "_DROP_" in next_item and not next_item.startswith("JUNCTION_"):
                        robot.queue = robot.queue[:-1] + ["check_slot", robot.queue[-1]]

                else:
                    robot.mode=next_item
                    if next_item == "collect":
                        # Plan collection point
                        # Find collection point route and enter. include "pick_up"     
                        for key in map.keys():
                            
                            if key.startswith("GREEN_"):
                                if map[key]["type"]=="DEAD_END":
                                    robot.drop_off_info[0]=0
                                    robot.final_objective=key
                                
                                    break
                            if key.startswith("RED_"):
                                if map[key]["type"]=="DEAD_END":
                                    robot.drop_off_info[0]=1
                                    robot.final_objective=key
                                
                                    break
                        else:
                            for key in map.keys():
                                if key.startswith(("GREEN_","RED_")):
                                    map[key]["type"]="DEAD_END"
                            robot.queue.append("collect")
                            robot.mode=""
                        
                        if not robot.mode == "":
                            robot.queue.append("pick_up")
                            print(robot.find_route(robot.final_objective))
                            robot.queue+=robot.find_route(robot.final_objective)
                            robot.mode=""
                            
                    elif next_item == "deliver":
                        # Plan deliverance point
                        # Find deliverance point route and enter. include "drop_off"
                        drop_off_string = ""
                        if robot.drop_off_info[0]==0:
                            drop_off_string+="LEFT_"
                        else:
                            drop_off_string+="RIGHT_"
                        if robot.drop_off_info[1]==0:
                            drop_off_string+="BOTTOM_"
                        else:
                            drop_off_string+="TOP_"
                        drop_off_string+="DROP_"
                    
                        for drop_number in range(1,7):
                            if map[drop_off_string+str(drop_number)]["type"]=="DEAD_END":
                                robot.final_objective=drop_off_string+str(drop_number)                            
                                break
                        else:
                            # If all of them are full, then reset
                            for key in map.keys():
                                if key.startswith(drop_off_string):
                                    map[key]["type"]="DEAD_END"
                            robot.queue.append("deliver")
                            robot.mode = ""
                        
                        if not robot.mode == "":
                            robot.queue.append("drop_off")
                            print(robot.find_route(robot.final_objective))
                            print("FINAL_OBJECTIVE: " + robot.final_objective)
                            robot.queue+=robot.find_route(robot.final_objective)
                            robot.mode=""
                            
                    elif next_item == "go_back":
                        robot.final_objective="START"
                        robot.queue.append("stop")
                        robot.find_route(robot.final_objective)
                        robot.queue+=robot.find_route(robot.final_objective)
                        robot.mode=""
                        
                    elif next_item == "straight":

                        robot.forward(motor3,motor4)
                    elif next_item == "reverse_straight":
                        print(robot.mode)
                        reversing=True
                        robot.forward(motor3,motor4, reverse=reversing)
                        robot.mode="straight"
                    elif next_item == "wait_light":
                        # ADDED (2026-10-20): stop in front of the traffic light
                        # and watch it; the "wait_light" mode below goes on
                        # when it is green.
                        robot.stop(motor3, motor4)
                        tasks.light_reset()
                        print("waiting for the traffic light")
                    elif next_item == "check_slot":
                        # ADDED (2026-10-20): the robot has turned to face a drop
                        # slot (see "check_slot" above). If the front distance
                        # sensor sees a box in it, mark the slot "OCCUPIED", drop
                        # the drive in ("straight") and the "drop_off" from the
                        # queue, and plan the delivery again: "deliver" picks the
                        # next free slot (when all are full it resets them, as
                        # before). The wheels are already on the junction, so the
                        # next turn starts without driving forwards (skip_forward).
                        robot.stop(motor3, motor4)
                        if tasks.slot_occupied():
                            print("slot %s already has a box (%s mm) - trying the next slot"
                                  % (robot.final_objective, tasks.last_slot_mm))
                            map[robot.final_objective]["type"]="OCCUPIED"
                            robot.queue.pop()          # "straight" (drive into the slot)
                            robot.queue.pop()          # "drop_off"
                            robot.queue.append("deliver")
                            skip_forward=True
                        robot.mode=""
            
            elif robot.mode == "straight":

                # Detection
                # CHANGED (2026-10-20): junctions are ignored until
                # holdoff_until (set when a junction is detected, see
                # JUNCTION_HOLDOFF_MS at the top; not set after a turn).
                if ((robot.left_sensor == 1 and robot.left_sensor_previous == 0)
                    or (robot.right_sensor == 1 and robot.right_sensor_previous == 0)) \
                        and now >= holdoff_until:
                    # A junction has been detected
                
                    print("JUNCTION DETECTED")
                    robot.previous_position=robot.position
                    print(robot.position)

                    robot.position=robot.objective
                    # print("ROBOT_OBJECTIVE"+robot.objective)
                    
                    # In case it wants to reverse
                    reversing=False
                    robot.mode="" # Moves onto next instruction (which is turning)
                    holdoff_until = now + JUNCTION_HOLDOFF_MS   # ADDED (2026-10-20)
                    next_forward_ms = None    # ADDED (2026-10-20): the sensors are on this junction
                # If not moving straight
                elif motor3.return_state() == 0 or motor4.return_state()==0:
                    robot.forward(motor3,motor4, reverse=reversing)
                    print("motor not forward")
                if not reversing:
                    corrections(robot,motor3,motor4, reverse=reversing)
            
                
            
            
            elif robot.mode == "correction":

                if robot.front_left_sensor == 1 and robot.front_right_sensor == 1:
                    robot.stop(motor3,motor4)
                    robot.mode="" # Moves back to straight or turning
                # Detect if there is a dead_end. If there is, then stop going straight.
                elif robot.front_left_sensor == 0 and robot.front_right_sensor == 0 and robot.queue[-1]=="straight":
                    #' I can specify further with more ifs
                    robot.stop(motor3,motor4)
                    robot.previous_position=robot.position
                    print(robot.position)

                    robot.position=robot.objective
                    robot.queue.pop()
                    robot.mode=""

            elif robot.mode == "turning":
                #robot.specific_mode is in form ["direction", degrees]
                # direction can be straight, left, right, about and degrees is the relevant degrees (90)
                # There are two ways we can do this. Either turn for a certain amount of time, or turn until fron sensors trigger
                # If straight, then go straight for a bit so that it doesn't immediately trigger junction hit again?
            
            
                # FIX (2026-10-15): the turn did not actually turn. Changes:
                #  1. Time units: `now` is in MILLISECONDS, so the old
                #     time_count=now+2 gave only 2 ms of driving forward and 2 ms
                #     of turning. Now: NODE_FORWARD_MS forward (or the 4th entry
                #     of the turning item, e.g. ["turning", "left", 90, 0]), and
                #     at least TURN_MIN_MS of turning.
                #  2. End of the turn: before, it ended as soon as both centre
                #     sensors saw a line, but at a junction they are still on the
                #     OLD line when the turn starts, so it ended at once. Now the
                #     centre sensors must first leave the old line (both black,
                #     left_old_line) and then both find the new line.
                #  3. Turn on the spot (spin(): one wheel forwards, one
                #     backwards) instead of Robot.left / Robot.right, which pivot
                #     about one stopped wheel and so move the robot off the
                #     junction while turning.
                #  4. corrections() is no longer called while driving forward
                #     onto the junction: it switched the mode to "correction" and
                #     queued the string "turning", which the queue code cannot
                #     run, so the robot got stuck.
                #  5. A "straight" turn now also resets `turning`; before, it
                #     stayed True and the NEXT turn skipped its start-up step.
                # CHANGED (2026-10-20): skip_forward (see the top of main()): no
                # driving forwards before this turn, the wheels are already on
                # the junction.
                if not turning:
                    # Turning hasn't started, do the turns
                    turning=True
                    robot.mode_extra_specific="move_straight"
                    
                    
                    if skip_forward:
                        forward_ms = 0
                    elif next_forward_ms is not None:
                        forward_ms = next_forward_ms
                    else:
                        forward_ms = NODE_FORWARD_MS
                    skip_forward = False
                    next_forward_ms = None
                    if robot.mode_specific[0]=="straight":
                        # MAYBE check if will trigger junction (need to implement)
                        robot.forward(motor3,motor4)
                        time_count=now+int(forward_ms/4)
                    elif robot.mode_specific[0]=="left":
                        robot.forward(motor3,motor4)
                        time_count=now+forward_ms
                    elif robot.mode_specific[0]=="right":
                        robot.forward(motor3,motor4)
                        time_count=now+forward_ms
                    elif robot.mode_specific[0]=="about":
                        robot.forward(motor3,motor4, reverse=True)
                        reversing=True
                        time_count=now+int(NODE_FORWARD_MS/4)
                    

                elif robot.mode_extra_specific=="move_straight":
                    # Make sure does not change off line
                    if now>=time_count:
                        robot.mode_extra_specific="move_turning"
                        reversing=False
                        left_old_line=False

                        # CHANGED (2026-10-20): TURN_MIN_MS stays where it is (top
                        # of this file) and is no longer changed here. Assigning
                        # to it inside main() made Python treat TURN_MIN_MS as a
                        # LOCAL variable of main() everywhere in main(), which
                        # only worked by luck. turn_min is the minimum turn time
                        # for THIS turn: START_TURN_MIN_MS for the first turn at
                        # the start (exact turn especially for the start).
                        turn_min = START_TURN_MIN_MS if at_start else TURN_MIN_MS
                        
                        
                        if robot.mode_specific[0]=="straight":
                            # MAYBE check if will trigger junction (need to implement)
                            robot.forward(motor3,motor4,reverse=reversing)
                            robot.mode=""
                            turning=False
                            # Move back to straight movement
                            # initial turn
                        elif robot.mode_specific[0]=="left":
                            robot.spin(motor3, motor4, "left", TURN_SPEED)
                            time_count=now+turn_min
                        elif robot.mode_specific[0]=="right":
                            robot.spin(motor3, motor4, "right", TURN_SPEED)
                            time_count=now+turn_min
                        elif robot.mode_specific[0]=="about":
                            robot.about(motor3,motor4)
                            time_count=now+turn_min*2

                        at_start=False


                elif robot.mode_extra_specific=="move_turning":
                    # If we need crash detection or anything here then do it
                    if robot.front_left_sensor == 0 and robot.front_right_sensor == 0:
                        left_old_line = True
                    if now<time_count or not left_old_line:
                        pass
                    else:
                        if robot.front_left_sensor == 1 and robot.front_right_sensor == 1:
                            robot.stop(motor3,motor4)
                            robot.mode="" # Moves to next step
                            robot.direction=functions.turn_to_direction(robot.direction, robot.mode_specific[0])
                            print(robot.direction)

                            turning=False
                            # CHANGED (2026-10-20): no junction hold-off after a
                            # turn (see JUNCTION_HOLDOFF_MS at the top).
                            holdoff_until = 0
                            # ADDED (2026-10-20): see CLOSE_AFTER_TURN.
                            next_forward_ms = after_turn_forward_ms
                            after_turn_forward_ms = None
            elif robot.mode == "pick_up":
                #Do some actual grabbing stuff etc
                robot.previous_position=robot.position
                robot.position=robot.objective
                print(robot.position)
                utime.sleep(1)
                print("GRABBING STUFF")
                
                
                
                
                # This should have the grabbing and so on code
                # robot.drop_off_info[1]=0
                # CHANGED (2026-10-20): the fixed 0 above was a placeholder for
                # the rack level. The level now comes from the magnet in the
                # box: read_pole() watches the magnet sensor for
                # config.MAG_READ_MS and returns "north", "south" or None;
                # config.MAG_POLE_LEVEL turns that into "TOP" / "BOTTOM"
                # (MAG_NO_POLE_LEVEL if no magnet was found). "deliver" reads
                # drop_off_info[1]: 0 -> BOTTOM_ slots, 1 -> TOP_ slots.
                pole = tasks.read_pole()
                level = tasks.level_for_pole(pole)
                robot.drop_off_info[1] = 1 if level == "TOP" else 0
                print("box magnet: %s (strongest field %s mV) -> %s level" % (
                    pole if pole else "NOT FOUND", tasks.last_field_mv, level))
                map[robot.final_objective]["type"]="COLLECTED"
                robot.mode=""
                robot.has_box=True
            
                pass
            elif robot.mode == "drop_off":
                #Do some actual grabbing stuff etc
                utime.sleep(2)
                print("DROPPING STUFF")
                # This should have the grabbing and so on code
                robot.mode_extra_specific="move_straight_and_detect"
                robot.forward(motor3,motor4)
                time_count=now+NODE_FORWARD_MS
            

                robot.box_count+=1
                robot.has_box=False
                
                robot.previous_position=robot.position
                robot.position=robot.objective
                map[robot.final_objective]["type"]="FILLED"
                
                robot.mode=""
                
                
            elif robot.mode == "wait_light":
                # ADDED (2026-10-20): stopped in front of the traffic light (see
                # "wait_light" above). Drive on when the IR receiver reports
                # green (IR beacon seen without a break for
                # config.TRAFFIC_IR_CONFIRM_MS). No time limit.
                if tasks.light_update() == "green":
                    print("traffic light green - go (waited %d ms)" % tasks.light_waited_ms())
                    robot.mode=""

            elif robot.mode == "stop":
                is_started=False
                robot.stop(motor3, motor4)
                print("REACHED START")
    




if __name__ == "__main__":
    # ADDED (2026-10-15, safety): always switch both drive motors off when the
    # program ends for ANY reason (Ctrl-C in the REPL, an exception, ...).
    # Without this the PWM outputs keep their last duty cycle and the wheels
    # keep turning after the program has stopped. GP5 and GP6 are the PWM
    # pins of motor3 (PWM1) and motor4 (PWM2); setting the duty to 0 stops them.
    try:
        main()
    finally:
        for pwm_pin in (5, 6):
            machine.PWM(machine.Pin(pwm_pin)).duty_u16(0)
        print("motors stopped")