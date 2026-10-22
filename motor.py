from machine import Pin, PWM
from utime import sleep
import functions

class Motor:
    def __init__(self, dirPin, PWMPin):
        self.mDir = Pin(dirPin, Pin.OUT)  # set motor direction pin
        self.pwm = PWM(Pin(PWMPin))  # set motor pwm pin
        self.pwm.freq(1000)  # set PWM frequency
        self.pwm.duty_u16(0)  # set duty cycle - 0=off
        self.forward_speed=100
        self.backward_speed=100
        
    def off(self):
        self.pwm.duty_u16(0)
        
    def Forward(self, speed=100):
        self.mDir.value(0)                     # forward = 0 reverse = 1 motor
        self.pwm.duty_u16(int(65535 * speed / 100))  # speed range 0-100 motor

    def Reverse(self, speed=100):
        self.mDir.value(1)
        self.pwm.duty_u16(int(65535 * speed / 100))
        
    def return_state(self):
        if self.pwm.duty_u16()==0:
            return 0
        elif self.mDir.value()==0:
            return 1
        elif self.mDir.value()==1:
            return -1
        else:
            return 1000000


def test_motors():
    motor3 = Motor(dirPin=4, PWMPin=5)  # Motor 3 is controlled from Motor Driv2 #1, which is on GP4/5
    motor4 = Motor(dirPin=7, PWMPin=6)  # Motor 4 is controlled from Motor Driv2 #1, which is on GP4/5
    
    
    robot=functions.Robot()
    
    
    # print("Forward")
    # robot.forward(motor3, motor4)
    # # from control.start_stop import StartStopButton
    # # button = StartStopButton.from_config()
    # # is_pressed=True
    # # try:
    # #     while True:
    # #         if not is_pressed and button.pressed():
    # #             robot.forward(motor3, motor4)
    # #             is_pressed=True
    # #             sleep(1)
    # #         elif is_pressed and button.pressed():
                
    # #             robot.forward(motor3, motor4, reverse=True)
    # #             is_pressed=False
    # #             sleep(1)
    # # finally:
    
    print("Forward")
    robot.forward(motor3, motor4)
    sleep(1)
    print("Right")
    robot.right(motor3, motor4)
    sleep(1)
    print("Left")
    robot.left(motor3, motor4)

    sleep(1)
    print("Reverse")
    robot.forward(motor3, motor4, reverse=True)
    sleep(1)
    robot.stop(motor3, motor4)
    
    # count=0
    # while count<1:
    #     count+=1
    #     print("Forward")
    #     motor3.Forward(100)
    #     motor4.Forward(100)

    #     sleep(2)
    #     print("Reverse")
    #     motor3.Reverse()
    #     motor4.Reverse()
    #     sleep(1)
    # motor3.off()
    # motor4.off()

if __name__ == "__main__":
    test_motors()