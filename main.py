import functions as f
import utime
import machine
import config
from actuators.servo import Servo
from actuators.leds import StatusLeds
from sensors.magnetic_baselining import sample_magnetic_baseline


import os
#Clear logfile?
logfile = open('print_log.txt', 'w')
logfile.close()

logfile = open('print_log.txt', 'a')
os.dupterm(logfile)
print("This will be saved to print_log.txt even if unplugged!")


def main():

    variables = f.initialise_main_variables()
    #initialise all the variables, map, tasks, robot, sensors, motors, servos
    button = variables[1]
    now = utime.ticks_diff(utime.ticks_ms(), variables[16].start)
    variables[29].show(yellow=True, red=False, green=False)
    
    f.wait_for_start(button)
    f.gripper_start(variables[19], variables[20])
    
    variables[16].baseline_data=sample_magnetic_baseline(26, 4)
    print(variables[16].baseline_data)
    
    variables[16].queue=[
        "deliver",
        "collect",
                            
    ]
    variables[16].position = "START"
    variables[16].objective = "START"
    variables[16].direction = "north"
    
    


    while True:

        variables=f.button_check(variables)
        #check for a button press
        is_started = variables[6]
        robot = variables[16]
        #fetch needed variables
        
        if is_started:
            
            now = utime.ticks_diff(utime.ticks_ms(), robot.start)
            if variables[2]%1==0:
                variables = f.update_sensors(variables)

            if len(robot.queue)==0:
                
                if robot.box_count >= 3:
                    variables[16].queue=[
                        "go_back", 
                        "deliver",
                        "collect",                                                                   
                    ]+variables[16].queue
                else:
                    variables[16].queue=[
                        "deliver",
                        "collect",                                            
                    ]+variables[16].queue
            
            
            variables, now = f.robot_mode_check(variables, now)
            
            # Action
            


if __name__ == "__main__":

    try:
        main()
    finally:
        for pwm_pin in (5, 6):
            machine.PWM(machine.Pin(pwm_pin)).duty_u16(0)
        for servo in [Servo(pin) for name, pin in config.SERVO_PINS.items()]:
            servo.set_angle(0)
            servo.off()
        
        StatusLeds.from_config().show(red=False, green=False, yellow=False)

        print("motors stopped")