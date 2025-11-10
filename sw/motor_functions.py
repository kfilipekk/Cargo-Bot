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
    set_motor_speed(speed, left_dir, speed, right_dir)

    ##Check if already on line to determine if we need two-phase turn
    sensor_state = sensors.read_all_sensors()
    need_to_leave_line = sensor_state[1] == 1 or sensor_state[2] == 1
    sensors_off_line = not need_to_leave_line

    try:
        ##Phase 1: Turn until off the current line
        if need_to_leave_line:
            while time.ticks_diff(time.ticks_ms(), start) < timeout_ms:
                sensor_state = sensors.read_all_sensors()
                if sensor_state[1] == 0 and sensor_state[2] == 0:
                    sensors_off_line = True
                    break
                time.sleep_ms(10)

        ##Phase 2: Keep turning until both center sensors detect new line
        if sensors_off_line:
            while time.ticks_diff(time.ticks_ms(), start) < timeout_ms:
                sensor_state = sensors.read_all_sensors()
                if sensor_state[1] == 1 and sensor_state[2] == 1:
                    stop_motors()
                    return True
                time.sleep_ms(10)
    finally:
        stop_motors()
    return False


def turn(angle=90, clockwise=True, speed=None):
    speed = speed if speed is not None else ROBOT_CONFIG.BASE_SPEED
    duration = int(ROBOT_CONFIG.TURN_90_TIME_MS * angle / 90) if clockwise else int(ROBOT_CONFIG.TURN_90_TIME_MS_CCW * angle / 90)
    set_motor_speed(speed, 1 if clockwise else 0, speed, 0 if clockwise else 1)
    time.sleep_ms(duration)
    stop_motors()

def turn_right(angle=90, speed=None):
    turn(angle, True, speed)

def turn_left(angle=90, speed=None):
    turn(angle, False, speed)




def set_motor_speed(left_speed, left_dir, right_speed, right_dir):
    ##Apply motor corrections and enforce speed limits for both motors
    for side, speed, dir_pin, pwm, corr in [
        ("left", left_speed, left_dir_pin, left_pwm, ROBOT_CONFIG.LEFT_MOTOR_CORRECTION),
        ("right", right_speed, right_dir_pin, right_pwm, ROBOT_CONFIG.RIGHT_MOTOR_CORRECTION)
    ]:
        val = int(speed * corr)
        ##Enforce minimum speed threshold to prevent stalling
        if val > 0 and val < ROBOT_CONFIG.MIN_SPEED:
            val = ROBOT_CONFIG.MIN_SPEED
        val = min(255, val)
        dir_pin.value(left_dir if side == "left" else right_dir)
        ##Convert 0-255 speed to 16-bit PWM duty cycle
        pwm.duty_u16(int(val * 65535 / 255))

def stop_motors():
    left_pwm.duty_u16(0)
    right_pwm.duty_u16(0)

def move(speed=255, direction=1, duration_ms=None):
    speed = speed if speed is not None else ROBOT_CONFIG.BASE_SPEED
    set_motor_speed(speed, direction, speed, direction)
    if duration_ms:
        time.sleep_ms(duration_ms)
        stop_motors()

if __name__ == "__main__":
    print("Testing motor functions...")
    turn_right(90)
    time.sleep(2)
    turn_left(90)
    time.sleep(2)
    stop_motors()
