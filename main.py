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
            robot.queue=["deliver","collect"]
        # Detection
        
        
        # Action
        match robot.mode:  
            case None:
                next_item=robot.queue.pop()
                if isinstance(next_item, list):
                    # If turning etc
                    robot.mode=next_item[0]
                    robot.mode_specific=next_item[1:]
                    functions.degrees_to_time(next_item[2]) # gets the count for that specific turn
                    
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
                pass
    
    pass




if __name__ == "__main__":
    main()