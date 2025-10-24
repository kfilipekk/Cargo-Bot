import time
from machine import Pin
from .motor_functions import set_motor_speed, stop_motors
from .constants import ROBOT_CONFIG

## Sensor Setup
sensor_1 = Pin(11, Pin.IN, Pin.PULL_DOWN)  ## Leftmost
sensor_2 = Pin(8, Pin.IN, Pin.PULL_DOWN)  ## Centre-left
sensor_3 = Pin(9, Pin.IN, Pin.PULL_DOWN)  ## Centre-right
sensor_4 = Pin(10, Pin.IN, Pin.PULL_DOWN)  ## Rightmost

def read_all_sensors(): return list(sensor.value() for sensor in (sensor_1, sensor_2, sensor_3, sensor_4))

## PID State (global dictionary to store state across function calls)
pid_state = {"last_error": 0.0,"integral": 0.0,"filtered_derivative": 0.0}

def follow_line_pid():
    """PID-based line following with 4 sensors."""
    s = read_all_sensors()
    # Error: -2 (far left) to +2 (far right), 0 is centered
    if not any(s):
        error = 0
        pid_state["integral"] *= 0.9
    elif s[1] and s[2]:
        error = 0
    elif s[0]:
        error = -2
    elif s[1] and not s[2]:
        error = -1
    elif not s[1] and s[2]:
        error = 1
    elif s[3]:
        error = 2
    else:
        error = pid_state["last_error"]
        pid_state["integral"] *= 0.5
    pid_state["integral"] = max(-ROBOT_CONFIG.PID_MAX_INTEGRAL, min(ROBOT_CONFIG.PID_MAX_INTEGRAL, pid_state["integral"] + error))
    raw_deriv = error - pid_state["last_error"]
    pid_state["filtered_derivative"] = ROBOT_CONFIG.PID_ALPHA * raw_deriv + (1 - ROBOT_CONFIG.PID_ALPHA) * pid_state["filtered_derivative"]
    correction = (ROBOT_CONFIG.PID_KP * error + ROBOT_CONFIG.PID_KI * pid_state["integral"] + ROBOT_CONFIG.PID_KD * pid_state["filtered_derivative"]) * ROBOT_CONFIG.PID_CORRECTION_FACTOR
    pid_state["last_error"] = error
    base = ROBOT_CONFIG.BASE_SPEED
    left = max(ROBOT_CONFIG.MIN_SPEED, min(255, int(base + correction)))
    right = max(ROBOT_CONFIG.MIN_SPEED, min(255, int(base - correction)))
    set_motor_speed(left, 1, right, 1)


def run_line_follower(mode="basic", debug=False):
    """Main function to run the line follower."""
    print(f"Starting line follower (mode: {mode})...\nPress Ctrl+C to stop.")
    try:
        if mode == "pid":
            follow_line_pid()
        else:
            print(f"Unknown mode: {mode}")
        if debug:
            vals = read_all_sensors()
            print(f"Sensors: {' '.join(map(str, vals))} | State: {vals}")
        time.sleep_ms(10)
    except KeyboardInterrupt:
        print("\nStopping line follower...")
        stop_motors()
        print("Stopped.")


if __name__ == "__main__":
    ## test_sensors()
    run_line_follower(mode="basic", debug=True)
