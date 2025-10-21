import time
from machine import Pin, PWM

# --- Pin Definitions ---
PWM1_PIN = 5
DIR1_PIN = 4
PWM2_PIN = 6
DIR2_PIN = 7

# --- Hardware Setup ---
dir1 = Pin(DIR1_PIN, Pin.OUT)
dir2 = Pin(DIR2_PIN, Pin.OUT)

pwm1 = PWM(Pin(PWM1_PIN))
pwm1.freq(1000)

pwm2 = PWM(Pin(PWM2_PIN))
pwm2.freq(1000)

# --- Main Loop ---
print("Starting motor control loop...")
while True:
    print("Ramping up speed (Direction 1)...")
    for value in range(0, 256, 5):
        dir1.low()
        dir2.low()

        # Convert 0-255 value to MicroPython's 16-bit duty cycle (0-65535)
        duty_cycle = int(value * 65535 / 255)

        pwm1.duty_u16(duty_cycle)
        pwm2.duty_u16(duty_cycle)
        time.sleep_ms(30)

    time.sleep(1)

    print("Ramping down speed (Direction 1)...")
    for value in range(255, -1, -5):
        dir1.low()
        dir2.low()

        duty_cycle = int(value * 65535 / 255)
        pwm1.duty_u16(duty_cycle)
        pwm2.duty_u16(duty_cycle)
        time.sleep_ms(30)

    time.sleep(1)

    print("Ramping up speed (Direction 2)...")
    for value in range(0, 256, 5):
        dir1.high()
        dir2.high()

        duty_cycle = int(value * 65535 / 255)
        pwm1.duty_u16(duty_cycle)
        pwm2.duty_u16(duty_cycle)
        time.sleep_ms(30)

    time.sleep(1)

    print("Ramping down speed (Direction 2)...")
    for value in range(255, -1, -5):
        dir1.high()
        dir2.high()

        duty_cycle = int(value * 65535 / 255)
        pwm1.duty_u16(duty_cycle)
        pwm2.duty_u16(duty_cycle)
        time.sleep_ms(30)

    time.sleep(1)