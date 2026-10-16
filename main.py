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
from sensors.line_processing import OUTER_LEFT, INNER_LEFT, INNER_RIGHT, OUTER_RIGHT
from control.start_stop import StartStopButton

# ADDED (2026-10-15): settings for the line-following bench test.
# LINE_TEST_ONLY = True: run the fixed TEST ROUTE below instead of the
#     deliver / collect planner. (The planner cannot run yet: with "collect"
#     it re-plans forever without moving.) Set it to False to run the full
#     task logic again; none of the test-route code below is used then.
#     The test route:
#       1. The robot starts ON a node (junction). Its first move is a turn
#          to the START_TURN side ("left") on the spot, then it follows the
#          line.
#       2. At every junction that has a branch to the RIGHT it turns right
#          into that branch. Junctions without a right branch are crossed
#          straight on.
#       3. If the line ends (all four sensors black) inside such a right
#          branch, i.e. the branch is a dead end, the robot reverses back to
#          the junction it came from and turns LEFT there, which puts it back
#          on the main road, heading the way it was going before, and it
#          carries on along the main road.
#       4. If the line ends anywhere else, or a turn / the reversing takes
#          too long, the robot stops (mode "stopped") and waits for the
#          start/stop button.
#     A press of the start/stop button stops the robot; the next press
#     starts the test route again from the beginning (step 1), so put the
#     robot back on the start node first.
# DRIVE_SPEED: percent of full motor power used by Robot.forward / left /
#     right / backward when no explicit speed is given. motor.Motor defaults
#     to 100 %, far too fast for tuning the line following. Start low and
#     raise it once the robot follows the line reliably.
LINE_TEST_ONLY = False
DRIVE_SPEED = 60

# ADDED (2026-10-15): tuning for turns, junctions and the test route.
# All times are in MILLISECONDS (`now` in main() counts milliseconds).
# START_TURN: first move at the start node: "left", "right" or None (no
#     turn, just follow the line).
# START_FORWARD_MS: drive forward this long before the start turn. 0 = the
#     wheels are already on the start node. If the start node is a little
#     in front of the wheels, raise it.
# NODE_FORWARD_MS: the line sensors sit in front of the wheels, so when the
#     sensors reach a junction the wheels are still short of it. The robot
#     drives on for this long so that the WHEELS are on the junction, then
#     turns on the spot. Roughly (distance from the line sensors to the wheel
#     axle) / (driving speed). If the robot ends up beside the new line
#     after turning (turned too early) raise it; if it overshoots the
#     junction lower it.
# TURN_SPEED: motor power (%) of both wheels while turning on the spot.
# TURN_MIN_MS: a turn never finishes earlier than this.
# TURN_TIMEOUT_MS: no line found within this time of turning -> stop.
# LINE_LOST_MS: all four line sensors black for this long = the line has
#     really ended (not just a short gap or a wobble).
# REVERSE_TIMEOUT_MS: reversing out of a dead end for longer than this
#     without finding the junction -> stop.
# JUNCTION_HOLDOFF_MS: after a turn, or after crossing a junction straight
#     on, ignore new junctions for this long so that the lines of the SAME
#     junction are not counted again.
START_TURN = "left"
START_FORWARD_MS = 0
NODE_FORWARD_MS = 1700
TURN_SPEED = 60
TURN_MIN_MS = 250
TURN_TIMEOUT_MS = 4000
LINE_LOST_MS = 150
REVERSE_TIMEOUT_MS = 5000
JUNCTION_HOLDOFF_MS = 400


def corrections(robot, motor3, motor4, motor_speed3=TURN_SPEED, motor_speed4=TURN_SPEED, reverse=None):
    if reverse:
        if robot.front_left_sensor == 1 and robot.front_right_sensor == 0:
            # If left front sensor does not detect and right does
            robot.queue.append(robot.mode)
            robot.mode="correction"
            robot.right(motor3,motor4,motor_speed3, motor_speed4, reverse=reverse)
            #Then turn right
            
        elif robot.front_left_sensor == 0 and robot.front_right_sensor == 1:
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
    # FIX (2026-10-15): left and right were swapped.
    # LineSensorArray.read() returns ONE 4-bit number:
    #     bit 3 = OUTER_LEFT  = Line Sensor "Edge Left"    (GP20, physical 26)
    #     bit 2 = INNER_LEFT  = Line Sensor "Centre Left"  (GP21, physical 27)
    #     bit 1 = INNER_RIGHT = Line Sensor "Centre Right" (GP19, physical 25)
    #     bit 0 = OUTER_RIGHT = Line Sensor "Edge Right"   (GP18, physical 24)
    # The old line
    #     ... = [bool(sensors.read() & (1 << i)) for i in range(4)]
    # gave bits 0, 1, 2, 3 (right to left) to left_sensor, front_left_sensor,
    # front_right_sensor, right_sensor, so a line under the Centre RIGHT
    # sensor set front_left_sensor = 1 and every correction steered AWAY from
    # the line. It also called read() four times per loop; one read is faster
    # and gives a consistent snapshot of all four sensors.
    # Values are 1 (sensor sees the white line) or 0 (black floor).
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





def all_black(robot):
    """ADDED (2026-10-15): True when none of the four line sensors sees the line."""
    return not (robot.left_sensor or robot.front_left_sensor
                or robot.front_right_sensor or robot.right_sensor)


def test_route_start():
    """ADDED (2026-10-15): task queue at the start of the test route.

    The queue is used from the END (robot.queue.pop()), so this runs the
    start turn first and then "straight" (follow the line). The 4th entry
    of the turning item is how long to drive forward before turning on the
    spot (see START_FORWARD_MS).
    """
    if START_TURN is None:
        return ["straight"]
    return ["straight", ["turning", START_TURN, 90, START_FORWARD_MS]]



def main():
    # Status booleans
    turning=False   # Checks if the turning sequence is running so we just start it once
    traffic=False   # Is there traffic? If so, stop all
    moving=False 
    reversing=False # Is the robot reversing or not?
    time_count=0    # How we keep track of a deadline timewise. Changed by saying time_count=now+3
    using_time=False    #Are we actually using the time right now or not?
    is_started=False    # Have we started
    
    
    # ADDED (2026-10-15): state for the fixed turns and the test route.
    turn_deadline=0       # turning: give up (stop) when now passes this
    left_old_line=False   # turning: True once both centre sensors have left the old line
    junction_right=False  # "junction" mode: a branch to the right has been seen
    in_branch=False       # test route: inside a right branch we turned into
    lost_since=None       # time at which all four sensors went black (None = line seen)
    holdoff_until=0       # ignore junctions until now reaches this
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
    for m in (motor3, motor4):
        m.forward_speed = DRIVE_SPEED
        m.backward_speed = DRIVE_SPEED

    sensors=line_sensor.LineSensorArray([20,21,19,18],True)

    # ADDED (2026-10-15, safety): start / stop button. Nothing moves until
    # the button is pressed (see wait_for_start above).
    button = StartStopButton.from_config()
    wait_for_start(button)
    # ADDED (2026-10-15): the test route begins with the start turn.
    if LINE_TEST_ONLY:
        # robot.queue = test_route_start()
        print("please activate LINE test in code")
        pass
    is_started=True
    #start running
    while True:
        if button.pressed():
            is_started=True
            print("Started")
        
        if is_started:
            # ADDED (2026-10-15, safety): a press of the start/stop button while
            # running (or the front bumping into something, since the button is a
            # crash sensor) stops both motors. The robot then waits for the next
            # press and starts again from a clean state (empty queue, mode "";
            # in test-route mode the route restarts with the start turn).
            if button.pressed():
                robot.stop(motor3, motor4)
                print("stopped by the start/stop button")
                utime.sleep(2)
                is_started=False
                robot.mode = ""
                robot.queue = test_route_start() if LINE_TEST_ONLY else []
                # The robot is put back on the start node facing the start
                # direction, so reset the heading it keeps track of; otherwise
                # the printed directions continue from the previous run.
                robot.direction = "north"
                turning = False
                in_branch = False
                lost_since = None

            now = utime.ticks_diff(utime.ticks_ms(), start)
            update_sensors(sensors, robot)

            # ADDED (2026-10-15, test route step 3 / 4): detect the end of the
            # line. Only while following the line ("straight" or "correction");
            # during turns, junctions and reversing all-black is normal.
            if LINE_TEST_ONLY and robot.mode in ("straight", "correction"):
                if not all_black(robot):
                    lost_since = None
                elif lost_since is None:
                    lost_since = now
                elif now - lost_since >= LINE_LOST_MS:
                    lost_since = None
                    if in_branch:
                        # Dead end inside the right branch: reverse straight
                        # back until all four sensors are on the main road line.
                        print("dead end - reversing to the junction")
                        in_branch = False
                        robot.queue = []   # drop the "straight" that corrections() may have queued
                        robot.mode = "reverse"
                        time_count = now + REVERSE_TIMEOUT_MS
                        robot.backward(motor3, motor4)
                    else:
                        print("line lost - stopped (press the button to restart)")
                        robot.stop(motor3, motor4)
                        robot.queue = []
                        robot.mode = "stopped"

            if robot.mode == "stopped":
                # ADDED (2026-10-15): the test route has stopped itself (see
                # above and the timeouts below). Wait for the button.
                continue

            if len(robot.queue)==0:
                if LINE_TEST_ONLY:
                    # ADDED (2026-10-15): line-following test only (see
                    # LINE_TEST_ONLY at the top): keep following the line.
                    robot.queue=["straight"]
                else:
                    robot.box_count+=1
                    robot.queue=["deliver","collect"]
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
                        robot.queue = robot.queue[:-2].append("reverse_straight")
                        
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
                        
                    elif next_item == "straight":
                        robot.forward(motor3,motor4)
                    elif next_item == "reverse_straight":
                        print(robot.mode)
                        robot.forward(motor3,motor4, reverse=reversing)
                        reversing=True
                        robot.mode="straight"

            elif robot.mode == "straight":
                # Detection
                # ADDED (2026-10-15): in test-route mode junctions are ignored for
                # JUNCTION_HOLDOFF_MS after a turn / crossing (see the top).
                if ((robot.left_sensor == 1 and robot.left_sensor_previous == 0)
                    or (robot.right_sensor == 1 and robot.right_sensor_previous == 0)) \
                        and not (LINE_TEST_ONLY and now < holdoff_until):
                    # A junction has been detected
                
                
                    robot.previous_position=robot.position
                    robot.position=robot.objective
                    print(robot.position) 
                    
                    # In case it wants to reverse
                    reversing=False
                    robot.mode="" # Moves onto next instruction (which is turning)
                    if LINE_TEST_ONLY:
                        # ADDED (2026-10-15, test route step 2): do not decide
                        # yet. Drive on over the junction in mode "junction" for
                        # NODE_FORWARD_MS (this also brings the wheels onto the
                        # junction for a turn on the spot) and watch the right
                        # outer sensor the whole time: if the robot reaches the
                        # junction at a slight angle, the right sensor may see
                        # the right branch a moment after the left one fired.
                        print("junction")
                        in_branch = False   # a junction inside a branch: it is not a simple dead end
                        robot.mode = "junction"
                        junction_right = robot.right_sensor == 1
                        time_count = now + NODE_FORWARD_MS
                        robot.forward(motor3, motor4)
                # If not moving straight
                elif motor3.return_state() == 0 or motor4.return_state()==0:
                    robot.forward(motor3,motor4, reverse=reversing)
                    print("motor not forward")

                corrections(robot,motor3,motor4, reverse=reversing)
            
                
            
            
            elif robot.mode == "correction":
                if robot.front_left_sensor == 1 and robot.front_right_sensor == 1:
                    robot.stop(motor3,motor4)
                    robot.mode="" # Moves back to straight or turning
                # Detect if there is a dead_end. If there is, then stop going straight.
                if robot.front_left_sensor == 0 and robot.front_right_sensor == 0 and robot.queue[-1]=="straight":
                    robot.stop(motor3,motor4)
                    robot.queue.pop()
                    robot.mode=""

            # elif robot.mode == "junction":
            #     # ADDED (2026-10-15, test route step 2): driving over a junction
            #     # (started in the "straight" branch above). After NODE_FORWARD_MS
            #     # the wheels are on the junction: turn right into a right branch
            #     # if one was seen, otherwise carry on straight.
            #     if robot.right_sensor == 1:
            #         junction_right = True
            #     if now >= time_count:
            #         if junction_right:
            #             print("right branch - turning right")
            #             in_branch = True
            #             # Already on the junction, so 0 ms forward before the turn.
            #             robot.queue = ["straight", ["turning", "right", 90, 0]]
            #             robot.mode = ""
            #         else:
            #             print("no right branch - straight on")
            #             robot.mode = "straight"
            #             holdoff_until = now + JUNCTION_HOLDOFF_MS

            elif robot.mode == "reverse":
                # ADDED (2026-10-15, test route step 3): reversing straight back
                # out of a dead-end right branch. The main road crosses the line
                # sensors side to side, so the robot is back at the junction when
                # ALL FOUR sensors see the white line at the same time (1111).
                # CHANGED (2026-10-15, bench test): before, it stopped as soon as
                # the two OUTER sensors had each seen white at some moment, even
                # at different moments; on the robot that ended the reversing
                # early, at an angle, with only one sensor on the line (1000).
                # Note: if the robot reverses at a clear angle the four sensors
                # may never be on the line together; then it keeps reversing
                # until REVERSE_TIMEOUT_MS and stops.
                # Then the sensors are on the main road and the wheels are past
                # it, so the normal turn (drive forward NODE_FORWARD_MS, then
                # turn on the spot) brings the wheels back onto the junction and
                # turns left, back onto the main road.
                if (robot.left_sensor == 1 and robot.front_left_sensor == 1
                        and robot.front_right_sensor == 1 and robot.right_sensor == 1):
                    print("back on the main road - turning left")
                    robot.stop(motor3, motor4)
                    robot.queue = ["straight", ["turning", "left", 90]]
                    robot.mode = ""
                elif now >= time_count:
                    print("reversing took too long - stopped (press the button to restart)")
                    robot.stop(motor3, motor4)
                    robot.queue = []
                    robot.mode = "stopped"

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
                #  6. If no line is found within TURN_TIMEOUT_MS (test route
                #     only) the robot stops instead of spinning forever.
                if not turning:
                    # Turning hasn't started, do the turns
                    turning=True
                    robot.mode_extra_specific="move_straight"
                    
                    if robot.mode_specific[0]=="straight":
                        # MAYBE check if will trigger junction (need to implement)
                        robot.forward(motor3,motor4)
                        time_count=now+NODE_FORWARD_MS
                    elif robot.mode_specific[0]=="left":
                        robot.forward(motor3,motor4)
                        time_count=now+NODE_FORWARD_MS
                    elif robot.mode_specific[0]=="right":
                        robot.forward(motor3,motor4)
                        time_count=now+NODE_FORWARD_MS
                    elif robot.mode_specific[0]=="about":
                        robot.forward(motor3,motor4, reverse=True)
                        reversing=False
                        time_count=now+NODE_FORWARD_MS

                elif robot.mode_extra_specific=="move_straight":
                    # Make sure does not change off line
                    if now>=time_count:
                        robot.mode_extra_specific="move_turning"
                        reversing=False
                        left_old_line=False
                        turn_deadline=now+TURN_TIMEOUT_MS
                        if robot.mode_specific[0]=="straight":
                            # MAYBE check if will trigger junction (need to implement)
                            robot.forward(motor3,motor4)
                            robot.mode=""
                            turning=False
                            # Move back to straight movement
                            # initial turn
                        elif robot.mode_specific[0]=="left":
                            robot.spin(motor3, motor4, "left", TURN_SPEED)
                            time_count=now+TURN_MIN_MS
                        elif robot.mode_specific[0]=="right":
                            robot.spin(motor3, motor4, "right", TURN_SPEED)
                            time_count=now+TURN_MIN_MS
                        elif robot.mode_specific[0]=="about":
                            robot.about(motor3,motor4)
                            time_count=now+TURN_MIN_MS*2

                elif robot.mode_extra_specific=="move_turning":
                    # If we need crash detection or anything here then do it
                    if robot.front_left_sensor == 0 and robot.front_right_sensor == 0:
                        left_old_line = True
                    if now<time_count or not left_old_line:
                        if LINE_TEST_ONLY and now >= turn_deadline:
                            print("no line found while turning - stopped (press the button to restart)")
                            robot.stop(motor3, motor4)
                            robot.queue = []
                            robot.mode = "stopped"
                            turning = False
                    else:
                        if robot.front_left_sensor == 1 and robot.front_right_sensor == 1:
                            robot.stop(motor3,motor4)
                            robot.mode="" # Moves to next step
                            robot.direction=functions.turn_to_direction(robot.direction, robot.mode_specific[0])
                            print(robot.direction)

                            turning=False
                            holdoff_until = now + JUNCTION_HOLDOFF_MS
                        elif LINE_TEST_ONLY and now >= turn_deadline:
                            print("no line found while turning - stopped (press the button to restart)")
                            robot.stop(motor3, motor4)
                            robot.queue = []
                            robot.mode = "stopped"
                            turning = False
            elif robot.mode == "pick_up":
                #Do some actual grabbing stuff etc
                utime.sleep(1)
                print("GRABBING STUFF")
                # This should have the grabbing and so on code
                robot.drop_off_info[1]=0
                map[robot.final_objective]["type"]="COLLECTED"
                robot.mode=""
            
                pass
            elif robot.mode == "drop_off":
                #Do some actual grabbing stuff etc
                utime.sleep(2)
                print("DROPPING STUFF")
                # This should have the grabbing and so on code
            
                map[robot.final_objective]["type"]="FILLED"
                robot.mode=""
    




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