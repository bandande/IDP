import functions
import MAP


def main():
    turning=False
    traffic=False
    moving=False
    time=0
    robot=functions.Robot()
    
    functions.reset_map()
    
    
    #start running
    while True:
        if len(robot.queue)==0:
            robot.box_count+=1
            robot.queue=["deliver","collect"]
        
        
            
        
        # Action
        match robot.mode:  
            case None:
                next_item=robot.queue.pop()
                if isinstance(next_item, list):
                    # If turning etc
                    robot.mode=next_item[0]
                    robot.mode_specific=next_item[1:]
                    functions.degrees_to_time(next_item[2]) # gets the count for that specific turn
                    # Write some code to get that time
                    
                elif next_item.isupper():
                    robot.queue+=robot.move_to(next_item)
                else:
                    match next_item:
                        case "collect":
                            # Plan collection point
                            # Find collection point route and enter. include "pick up"              
                            pass
                        case "deliver":
                            # Plan deliverance point
                            # Find deliverance point route and enter. include "drop_off"
                            pass
                        case "straight":
                            robot.motor_left == True 
                            robot.motor_right == True
            case "straight":
                # Detection
                if (robot.left_sensor == 1 and robot.left_sensor_previous == 0) or (robot.right_sensor == 1 and robot.right_sensor_previous == 0):
                    # A junction has been detected
                    robot.motor_left = False
                    robot.motor_right = True
                    ##=====Can be commented out
                    while time<50: # CHANGE THIS TO ACTUAL TIME
                        pass# This should prevenet double triggering of junctions and allow the type of junction to be seen. This can be adjusted
                    if robot.left_sensor == 1:
                        print("Can turn left")
                    if robot.right_sensor == 1:
                        print("Can turn right")
                    ##=====
                    robot.mode=None # Moves onto next instruction (which is turning)
                                        
                elif robot.front_left_sensor == 0 and robot.front_right_sensor == 1:
                    # If left front sensor does not detect and right does
                    robot.queue.append(robot.mode)
                    robot.mode="correction"
                    robot.motor_left = False
                    robot.motor_right = True
                    #Then turn right
                    
                elif robot.front_left_sensor == 1 and robot.front_right_sensor == 0:
                    # If left front sensor does not detect and right does
                    robot.queue.append(robot.mode)
                    robot.mode="correction"
                    robot.motor_left = True
                    robot.motor_right = False
                    #Then turn left
                    
                # If not moving straight
                elif robot.motor_left == False or robot.motor_right == False:
                    robot.motor_left = True 
                    robot.motor_right = True
            
            case "correction":
                if robot.front_left_sensor == 1 and robot.front_right_sensor == 1:
                    robot.motor_left = False
                    robot.motor_right = False
                    robot.mode=None # Moves back to straight/turning
                    
            
            case "turning":
                #robot.specific_mode is in form ["direction", degrees]
                # direction can be straight, left, right, about and degrees is the relevant degrees (90)
                # There are two ways we can do this. Either turn for a certain amount of time, or turn until fron sensors trigger
                # If straight, then go straight for a bit so that it doesn't immediately trigger junction hit again?
                pass
    




if __name__ == "__main__":
    main()