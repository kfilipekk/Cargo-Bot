
import time
from machine import Pin, PWM
from sw.constants import ROBOT_CONFIG
from sw import sensors

LEFT_MOTOR_PWM_PIN = 5
LEFT_MOTOR_DIR_PIN = 4
RIGHT_MOTOR_PWM_PIN = 6
RIGHT_MOTOR_DIR_PIN = 7

left_dir_pin = Pin(LEFT_MOTOR_DIR_PIN, Pin.OUT)
right_dir_pin = Pin(RIGHT_MOTOR_DIR_PIN, Pin.OUT)

left_pwm = PWM(Pin(LEFT_MOTOR_PWM_PIN))
left_pwm.freq(1000)

right_pwm = PWM(Pin(RIGHT_MOTOR_PWM_PIN))
right_pwm.freq(1000)

def turn_until_line_on_sensors(direction, speed=255, timeout_ms=3000):

    left_dir, right_dir = (0, 1) if direction == "left" else (1, 0)
    start = time.ticks_ms()
    ## Start continuous turn
    set_motor_speed(speed, left_dir, speed, right_dir)
    try:
        while time.ticks_diff(time.ticks_ms(), start) < timeout_ms:
            sensor_state = sensors.read_all_sensors()
            if sensor_state[1] == 1 and sensor_state[2] == 1:
                stop_motors()
                return True
            # small sleep to yield CPU and avoid busy-wait
            time.sleep_ms(10)
    finally:
        stop_motors()
    return False

def set_motor_speed(left_speed, left_dir, right_speed, right_dir):
    ## Apply correction, min/max, and set direction and PWM for both motors
    for side, speed, dir_pin, pwm, corr in [
        ("left", left_speed, left_dir_pin, left_pwm, ROBOT_CONFIG.LEFT_MOTOR_CORRECTION),
        ("right", right_speed, right_dir_pin, right_pwm, ROBOT_CONFIG.RIGHT_MOTOR_CORRECTION)
    ]:
        val = int(speed * corr)
        if val > 0 and val < ROBOT_CONFIG.MIN_SPEED:
            val = ROBOT_CONFIG.MIN_SPEED
        val = min(255, val)
        dir_pin.value(left_dir if side == "left" else right_dir)
        pwm.duty_u16(int(val * 65535 / 255))

def stop_motors():
    left_pwm.duty_u16(0)
    right_pwm.duty_u16(0)

def move(speed=255, direction=1, duration_ms=None):
    """Move forward (direction=1) or backward (direction=0)"""
    speed = speed if speed is not None else ROBOT_CONFIG.BASE_SPEED
    set_motor_speed(speed, direction, speed, direction)
    if duration_ms:
        time.sleep_ms(duration_ms)
        stop_motors()

if __name__ == "__main__":
    stop_motors()
