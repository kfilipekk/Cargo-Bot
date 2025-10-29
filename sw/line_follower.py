import time
from machine import Pin
from .motor_functions import set_motor_speed, stop_motors
from .constants import ROBOT_CONFIG

## Sensor Setup
sensor_1 = Pin(10, Pin.IN, Pin.PULL_DOWN)  ## Leftmost
sensor_2 = Pin(8, Pin.IN, Pin.PULL_DOWN)  ## Centre-left
sensor_3 = Pin(9, Pin.IN, Pin.PULL_DOWN)  ## Centre-right
sensor_4 = Pin(11, Pin.IN, Pin.PULL_DOWN)  ## Rightmost

def read_all_sensors(): return list(sensor.value() for sensor in (sensor_1, sensor_2, sensor_3, sensor_4))

## PID State (global dictionary to store state across function calls)
pid_state = {"last_error": 0.0, "integral": 0.0, "filtered_derivative": 0.0, "lost_line_count": 0}

def follow_line_pid():
    """PID-based line following with 4 sensors and line loss recovery."""
    s = read_all_sensors()

    ## Line completely lost (all sensors off)
    if not any(s):
        pid_state["lost_line_count"] += 1

        ## Use last known error to continue in same direction
        if pid_state["lost_line_count"] < 5:
            error = pid_state["last_error"] * 1.5  ## Amplify to search more aggressively
            pid_state["integral"] *= 0.8
        else:
            ## Extended line loss - recovery maneuver based on last direction
            error = pid_state["last_error"] * 3.0  ## Sharp turn to find line
            pid_state["integral"] = 0

    ## Line found - reset lost counter
    else:
        pid_state["lost_line_count"] = 0

        ## Both center sensors on line - going straight
        if s[1] and s[2]:
            error = 0
        ## Only left center off, right center on - veering right, turn left
        elif s[1] == 0 and s[2] == 1:
            error = -1
        ## Only right center off, left center on - veering left, turn right
        elif s[1] == 1 and s[2] == 0:
            error = 1
        ## Both center sensors off (but outer sensors may be on for intersection)
        ## Continue in last known direction
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


def run_line_follower(mode="pid", debug=False):
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
    run_line_follower(mode="pid", debug=True)
