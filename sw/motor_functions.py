import time
from machine import Pin, PWM

## Pin Definitions
PWM1_PIN = 5
DIR1_PIN = 4
PWM2_PIN = 6
DIR2_PIN = 7

## Hardware Setup
dir1 = Pin(DIR1_PIN, Pin.OUT)
dir2 = Pin(DIR2_PIN, Pin.OUT)

pwm1 = PWM(Pin(PWM1_PIN))
pwm1.freq(1000)

pwm2 = PWM(Pin(PWM2_PIN))
pwm2.freq(1000)

## Calibration Constants - adjust based on your robot
TURN_90_TIME_MS = 500
FORWARD_SPEED = 200
TURN_SPEED = 180


def set_motor_speed(motor1_speed, motor1_dir, motor2_speed, motor2_dir):
    ## Set speed (0-255) and direction (0/1) for both motors
    if motor1_dir:
        dir1.high()
    else:
        dir1.low()
    
    if motor2_dir:
        dir2.high()
    else:
        dir2.low()
    
    duty_cycle1 = int(motor1_speed * 65535 / 255)
    duty_cycle2 = int(motor2_speed * 65535 / 255)
    
    pwm1.duty_u16(duty_cycle1)
    pwm2.duty_u16(duty_cycle2)


def stop_motors():
    pwm1.duty_u16(0)
    pwm2.duty_u16(0)


def move_forward(speed=FORWARD_SPEED, duration_ms=None):
    set_motor_speed(speed, 0, speed, 0)
    
    if duration_ms is not None:
        time.sleep_ms(duration_ms)
        stop_motors()


def move_backward(speed=FORWARD_SPEED, duration_ms=None):
    set_motor_speed(speed, 1, speed, 1)
    
    if duration_ms is not None:
        time.sleep_ms(duration_ms)
        stop_motors()


def turn_left(angle=90, speed=TURN_SPEED):
    ## Turn left (counterclockwise) by specified angle
    duration = int(TURN_90_TIME_MS * angle / 90)
    set_motor_speed(speed, 1, speed, 0)
    time.sleep_ms(duration)
    stop_motors()


def turn_right(angle=90, speed=TURN_SPEED):
    ## Turn right (clockwise) by specified angle
    duration = int(TURN_90_TIME_MS * angle / 90)
    set_motor_speed(speed, 0, speed, 1)
    time.sleep_ms(duration)
    stop_motors()


def turn_90_clockwise(speed=TURN_SPEED):
    turn_right(90, speed)


def turn_90_counterclockwise(speed=TURN_SPEED):
    turn_left(90, speed)


def turn_180_clockwise(speed=TURN_SPEED):
    turn_right(180, speed)


def turn_180_counterclockwise(speed=TURN_SPEED):
    turn_left(180, speed)


def turn_270_clockwise(speed=TURN_SPEED):
    turn_right(270, speed)


def turn_270_counterclockwise(speed=TURN_SPEED):
    turn_left(270, speed)


def pivot_left(speed=TURN_SPEED, duration_ms=None):
    ## Pivot left (left motor stopped, right motor forward)
    set_motor_speed(0, 0, speed, 0)
    
    if duration_ms is not None:
        time.sleep_ms(duration_ms)
        stop_motors()


def pivot_right(speed=TURN_SPEED, duration_ms=None):
    ## Pivot right (right motor stopped, left motor forward)
    set_motor_speed(speed, 0, 0, 0)
    
    if duration_ms is not None:
        time.sleep_ms(duration_ms)
        stop_motors()


def ramp_speed(target_speed, direction=0, ramp_time_ms=1000):
    ## Gradually ramp up or down to target speed
    steps = 50
    delay = ramp_time_ms // steps
    
    for i in range(steps + 1):
        current_speed = int(target_speed * i / steps)
        set_motor_speed(current_speed, direction, current_speed, direction)
        time.sleep_ms(delay)


if __name__ == "__main__":
    print("Testing motor functions...")
    
    print("Moving forward...")
    move_forward(duration_ms=2000)
    time.sleep(1)
    
    print("Moving backward...")
    move_backward(duration_ms=2000)
    time.sleep(1)
    
    print("Turning 90° clockwise...")
    turn_90_clockwise()
    time.sleep(1)
    
    print("Turning 90° counterclockwise...")
    turn_90_counterclockwise()
    time.sleep(1)
    
    print("Turning 180°...")
    turn_180_clockwise()
    time.sleep(1)
    
    print("Turning 270° counterclockwise...")
    turn_270_counterclockwise()
    time.sleep(1)
    
    print("Test complete!")
    stop_motors()
