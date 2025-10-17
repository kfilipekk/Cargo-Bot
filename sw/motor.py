from machine import Pin , PWM
from utime import sleep

# Motor A
led = Pin(25,Pin.OUT)
ina1 = Pin(5,Pin.OUT)
ina2 = Pin(4, Pin.OUT)
pwma = PWM(Pin(5))
pwma.freq(1000)

# Motor B
inb1 = Pin(6, Pin.OUT)
inb2 = Pin(7, Pin.OUT)
pwmb = PWM(Pin(6))
pwmb.freq(1000)

led.toggle()


def RotateCW(duty):
    ina1.value(1)
    ina2.value(0)
    inb1.value(1)
    inb2.value(0)
    duty_16 = int((duty*65536)/100)
    pwma.duty_u16(duty_16)
    pwmb.duty_u16(duty_16)

def RotateCCW(duty):
    ina1.value(0)
    ina2.value(1)
    inb1.value(0)
    inb2.value(1)
    duty_16 = int((duty*65536)/100)
    pwma.duty_u16(duty_16)
    pwmb.duty_u16(duty_16)

def StopMotor():
    ina1.value(0)
    ina2.value(0)
    inb1.value(0)
    inb2.value(0)
    pwma.duty_u16(0)
    pwmb.duty_u16(0)


while True:
    duty_cycle=float(input("Enter pwm duty cycle"))
    print (duty_cycle)
    RotateCW(duty_cycle)
    sleep(5)
    RotateCCW(duty_cycle)
    sleep(5)
    StopMotor()
    sleep(1)

