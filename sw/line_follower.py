import time
from machine import Pin
from motor_functions import set_motor_speed, stop_motors

## Pin Definitions - Adjust based on your hardware
LEFT_SENSOR_PIN = 18
RIGHT_SENSOR_PIN = 19

## Line Following Parameters
BASE_SPEED = 150
TURN_SPEED = 100
CORRECTION_SPEED = 180

## Sensor Setup
left_sensor = Pin(LEFT_SENSOR_PIN, Pin.IN, Pin.PULL_DOWN)
right_sensor = Pin(RIGHT_SENSOR_PIN, Pin.IN, Pin.PULL_DOWN)


def read_sensors():
    ## Read both line sensors (0 = dark line, 1 = light surface)
    left = left_sensor.value()
    right = right_sensor.value()
    return left, right


def get_line_position():
    ## Determine robot position relative to line
    left, right = read_sensors()

    if left == 0 and right == 0:
        return 'on_line'
    elif left == 1 and right == 0:
        return 'left_of_line'
    elif left == 0 and right == 1:
        return 'right_of_line'
    else:
        return 'off_line'

def follow_line_basic():
    ## Basic line following with simple differential steering
    position = get_line_position()

    if position == 'on_line':
        set_motor_speed(BASE_SPEED, 0, BASE_SPEED, 0)

    elif position == 'left_of_line':
        set_motor_speed(TURN_SPEED, 0, CORRECTION_SPEED, 0)

    elif position == 'right_of_line':
        set_motor_speed(CORRECTION_SPEED, 0, TURN_SPEED, 0)

    else:
        stop_motors()


def follow_line_pid(kp=1.5, ki=0.01, kd=0.8):
    ## PID-based line following with anti-windup and derivative filtering
    if not hasattr(follow_line_pid, 'last_error'):
        follow_line_pid.last_error = 0
        follow_line_pid.integral = 0
        follow_line_pid.filtered_derivative = 0

    left, right = read_sensors()

    ## Calculate error with weighted positions for smoother tracking
    if left == 0 and right == 0:
        error = 0
        follow_line_pid.integral *= 0.9  ## Reduce integral when on line
    elif left == 1 and right == 0:
        error = -1
    elif left == 0 and right == 1:
        error = 1
    else:
        ## Both sensors off line - maintain last direction but don't accumulate integral
        error = follow_line_pid.last_error
        follow_line_pid.integral *= 0.5  ## Decay integral when lost

    ## Anti-windup: limit integral term to prevent overshooting
    follow_line_pid.integral += error
    max_integral = 10
    follow_line_pid.integral = max(-max_integral, min(max_integral, follow_line_pid.integral))

    ## Derivative with low-pass filter to reduce noise
    raw_derivative = error - follow_line_pid.last_error
    alpha = 0.3  ## Filter coefficient (0-1, lower = more filtering)
    follow_line_pid.filtered_derivative = alpha * raw_derivative + (1 - alpha) * follow_line_pid.filtered_derivative

    ## Calculate PID correction
    correction = kp * error + ki * follow_line_pid.integral + kd * follow_line_pid.filtered_derivative
    follow_line_pid.last_error = error

    ## Apply correction with smoother scaling
    correction_factor = 40  ## Reduced from 50 for gentler corrections
    left_speed = int(BASE_SPEED + correction * correction_factor)
    right_speed = int(BASE_SPEED - correction * correction_factor)

    ## Clamp speeds with minimum speed to prevent stalling
    min_speed = 60
    left_speed = max(min_speed, min(255, left_speed))
    right_speed = max(min_speed, min(255, right_speed))

    set_motor_speed(left_speed, 0, right_speed, 0)


def follow_line_smooth():
    ## Smooth line following with gradual corrections
    position = get_line_position()

    if position == 'on_line':
        set_motor_speed(BASE_SPEED, 0, BASE_SPEED, 0)

    elif position == 'left_of_line':
        left_speed = int(BASE_SPEED * 0.6)
        right_speed = int(BASE_SPEED * 1.0)
        set_motor_speed(left_speed, 0, right_speed, 0)

    elif position == 'right_of_line':
        left_speed = int(BASE_SPEED * 1.0)
        right_speed = int(BASE_SPEED * 0.6)
        set_motor_speed(left_speed, 0, right_speed, 0)

    else:
        set_motor_speed(TURN_SPEED, 0, TURN_SPEED, 0)


def calibrate_sensors(duration_seconds=3):
    ##Calibrate sensors by reading values over time
    print("Calibrating sensors...")
    print("Move robot over line and surface")

    left_min, left_max = 1, 0
    right_min, right_max = 1, 0

    start_time = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start_time) < duration_seconds * 1000:
        left, right = read_sensors()

        left_min = min(left_min, left)
        left_max = max(left_max, left)
        right_min = min(right_min, right)
        right_max = max(right_max, right)

        print(f"Left: {left}, Right: {right}")
        time.sleep_ms(100)

    print(f"\nCalibration complete:")
    print(f"Left sensor - Min: {left_min}, Max: {left_max}")
    print(f"Right sensor - Min: {right_min}, Max: {right_max}")


def test_sensors():
    ##Test function to continuously read and display sensor values
    ##Should be 0000 from starting point
    pass


def run_line_follower(mode='basic', debug=False):
    ## Main function to run the line follower (mode: 'basic', 'smooth', or 'pid')
    print(f"Starting line follower (mode: {mode})...")
    print("Press Ctrl+C to stop")

    try:
        while True:
            if mode == 'basic':
                follow_line_basic()
            elif mode == 'smooth':
                follow_line_smooth()
            elif mode == 'pid':
                follow_line_pid(kp=1.5, ki=0.01, kd=0.8)
            else:
                print(f"Unknown mode: {mode}")
                break

            if debug:
                left, right = read_sensors()
                print(f"L: {left}, R: {right}")

            time.sleep_ms(10)

    except KeyboardInterrupt:
        print("\nStopping line follower...")
        stop_motors()
        print("Stopped")


if __name__ == "__main__":
    ## Test sensors first: test_sensors()
    ## Calibrate if needed: calibrate_sensors()
    ## Run line follower:
    run_line_follower(mode='basic', debug=True)
