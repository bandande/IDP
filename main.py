import functions
import MAP
import utime
import motor
import sensors.line_sensor as line_sensor


def corrections(robot, motor3, motor4, motor_speed3=None, motor_speed4=None, reverse=None):
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
    robot.left_sensor,robot.front_left_sensor,robot.front_right_sensor,robot.right_sensor= [bool(sensors.read() & (1 << i)) for i in range(4)]
    
    
def main():
    turning=False
    traffic=False
    moving=False
    reversing=False
    time_count=0
    using_time=False
    robot=functions.Robot()
    print(robot.position)
    functions.reset_map()
    map=MAP.MAP.copy()
    # Measure elapsed time
    start = utime.ticks_ms()
    # elapsed = utime.ticks_diff(utime.ticks_ms(), start)
    
    motor3 = motor.Motor(dirPin=4, PWMPin=5)  # Motor 3 is controlled from Motor Driv2 #1, which is on GP4/5
    motor4 = motor.Motor(dirPin=7, PWMPin=6)  # Motor 4 is controlled from Motor Driv2 #1, which is on GP4/5
    
    sensors=line_sensor.LineSensorArray([20,21,19,18],True)
    
    #start running
    while True:
        now = utime.ticks_diff(utime.ticks_ms(), start)
        update_sensors(sensors, robot)
            
        if len(robot.queue)==0:
            robot.box_count+=1
            robot.queue=["deliver","collect"]
            # robot.queue=["straight"]
        
        
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
                robot.queue+=robot.move_to(next_item)
            else:
                robot.queue.append(next_item)
                if next_item == "collect":
                    # Plan collection point
                    # Find collection point route and enter. include "pick_up"     
                    for key in map.keys():
                        if key.startswith("GREEN"):
                            if map[key]["type"]=="DEAD_END":
                                robot.drop_off_info[0]=0
                                robot.final_objective=key
                                
                                break
                        if key.startswith("RED"):
                            if map[key]["type"]=="DEAD_END":
                                robot.drop_off_info[0]=1
                                robot.final_objective=key
                                
                                break
                    else:
                        for key in map.keys():
                            if key.startswith(("GREEN","RED")):
                                map[key]["type"]="DEAD_END"
                        robot.queue.append("collect")
                        robot.mode=""
                        
                    if not robot.mode == "":
                        robot.queue.append("pick_up")
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
                    
                    for drop_number in range(6):
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
                        robot.queue+=robot.find_route(robot.final_objective)
                        robot.mode=""
                        
                elif next_item == "straight":
                    robot.forward(motor3,motor4)
                    
        elif robot.mode == "straight":
            # Detection
            if ((robot.left_sensor == 1 and robot.left_sensor_previous == 0) 
                or (robot.right_sensor == 1 and robot.right_sensor_previous == 0)):
                # A junction has been detected
                
                # ##=====Can be commented out
                # robot.forward(motor3,motor4)
                # time_count=now+50
                # while now<time_count: # CHANGE THIS TO ACTUAL TIME
                #     pass# This should prevenet double triggering of junctions and allow the type of junction to be seen. This can be adjusted
                # if robot.left_sensor == 1:
                #     print("Can turn left")
                # if robot.right_sensor == 1:
                #     print("Can turn right")
                # robot.stop(motor3,motor4)
                # ##=====
                
                ##=====Can be commented out
                # THis provides the most basic logic to turning
                # if robot.left_sensor == 1:
                #     robot.queue.append(["turning", "left", 90])
                # if robot.right_sensor == 1:
                #     robot.queue.append(["turning", "right", 90])
                
                ##=====
                
                robot.previous_position=robot.position
                robot.position=robot.objective
                robot.mode="" # Moves onto next instruction (which is turning)
            # If not moving straight
            elif motor3.return_state() == 0 or motor4.return_state()==0:
                robot.forward(motor3,motor4)
                print("motor not forward")
            # Detect and make corrections
            corrections(robot,motor3,motor4, reverse=reversing)               
            
                
            
            
        elif robot.mode == "correction":
            if robot.front_left_sensor == 1 and robot.front_right_sensor == 1:
                robot.stop(motor3,motor4)
                robot.mode="" # Moves back to straight or turning
                    
        elif robot.mode == "turning":
            #robot.specific_mode is in form ["direction", degrees]
            # direction can be straight, left, right, about and degrees is the relevant degrees (90)
            # There are two ways we can do this. Either turn for a certain amount of time, or turn until fron sensors trigger
            # If straight, then go straight for a bit so that it doesn't immediately trigger junction hit again?
            
            
            if not turning:
                # Turning hasn't started, do the turns
                turning=True
                robot.mode_extra_specific="move_straight"
                if robot.mode_specific[0]=="straight":
                    # MAYBE check if will trigger junction (need to implement)
                    robot.forward(motor3,motor4)
                    robot.mode=""
                elif robot.mode_specific[0]=="left":
                    robot.forward(motor3,motor4)
                    time_count=now+2
                elif robot.mode_specific[0]=="right":
                    robot.forward(motor3,motor4)
                    time_count=now+2
                elif robot.mode_specific[0]=="about":
                    robot.forward(motor3,motor4, reverse=True)
                    reversing=True
                    time_count=now+2
                    
            elif robot.mode_extra_specific=="move_straight":
                # Make sure does not change off line                
                if now>time_count:
                    robot.mode_extra_specific="move_turning"
                    reversing=False
                    if robot.mode_specific[0]=="straight":
                        # MAYBE check if will trigger junction (need to implement)
                        robot.forward(motor3,motor4)
                        robot.mode=""
                        # Move back to straight movement
                        # initial turn
                    elif robot.mode_specific[0]=="left":
                        robot.left(motor3,motor4)
                        time_count=now+2
                    elif robot.mode_specific[0]=="right":
                        robot.right(motor3,motor4)
                        time_count=now+2
                    elif robot.mode_specific[0]=="about":
                        robot.about(motor3,motor4)
                        time_count=now+3
                    
                corrections(robot,motor3,motor4, reverse=reversing)
            elif robot.mode_extra_specific=="move_turning":
                # If we need crash detection or anything here then do it
                if now<time_count:
                    pass
                else:
                    if robot.front_left_sensor == 1 and robot.front_right_sensor == 1:
                        robot.stop(motor3,motor4)
                        robot.mode="" # Moves to next step
                        robot.direction=functions.turn_to_direction(robot.direction, robot.mode_specific[0])
                        print(robot.direction)
                        
                        turning=False
        elif robot.mode == "pick_up":
            #Do some actual grabbing stuff etc
            utime.sleep(1)
            # This should have the grabbing and so on code
            robot.drop_off_info[1]=1
            map[robot.final_objective]["type"]="COLLECTED"
            robot.mode=""
            
            pass
        elif robot.mode == "drop_off":
            #Do some actual grabbing stuff etc
            utime.sleep(2)
            # This should have the grabbing and so on code
            
            map[robot.final_objective]["type"]="FILLED"
            robot.mode=""
    




if __name__ == "__main__":
    main()