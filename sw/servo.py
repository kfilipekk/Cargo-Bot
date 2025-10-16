from machine import Pin, PWM
from utime import sleep

class Servo:
    def __init__(self, PWMPin):
        self.pwm = PWM(Pin(PWMPin))
        self.pwm.freq(50)

    def set_angle(self, angle):
        # angle is 0 to 180
        # pulse width from 500us to 2500us
        # 50Hz freq -> 20ms period
        # duty_u16 converts 0-65535 to 0-100% duty cycle
        # 500us is 500/20000 = 2.5% duty cycle. 0.025 * 65535 = 1638
        # 2500us is 2500/20000 = 12.5% duty cycle. 0.125 * 65535 = 8192
        min_duty = 1638
        max_duty = 8192
        duty = min_duty + (max_duty - min_duty) * (angle / 180)
        self.pwm.duty_u16(int(duty))

    def off(self):
        self.pwm.duty_u16(0)

def test_servo():
    servo = Servo(PWMPin=13)

    while True:
        print("Setting angle to 0")
        servo.set_angle(0)
        sleep(2)
        
        print("Setting angle to 90")
        servo.set_angle(90)
        sleep(2)
        
        print("Setting angle to 180")
        servo.set_angle(180)
        sleep(2)

        print("Setting angle to 270")
        servo.set_angle(270)
        sleep(2)

if __name__ == "__main__":
    test_servo()
