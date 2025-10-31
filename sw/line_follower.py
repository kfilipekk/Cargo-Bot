import time
from machine import Pin
from .motor_functions import set_motor_speed, stop_motors
from .constants import ROBOT_CONFIG
from .sensors import sensor_state

## PID State (global dictionary to store state across function calls)
pid_state = {"last_error": 0.0,"integral": 0.0,"filtered_derivative": 0.0}

def follow_line_basic():
    """Basic line following using only center sensors s[1] and s[2]."""
    s = sensor_state
    base_speed = ROBOT_CONFIG.BASE_SPEED
    turn_speed = ROBOT_CONFIG.TURN_SPEED

    if s[1] and s[2]:
        set_motor_speed(base_speed, 1, base_speed, 1)
    elif s[1]:
        set_motor_speed(turn_speed, 1, base_speed, 1)
    elif s[2]:
        set_motor_speed(base_speed, 1, turn_speed, 1)
    else:
        stop_motors()

def follow_line_pid():
    """PID-based line following with 4 sensors."""
    s = sensor_state
    if not any(s):
        error = 0
        pid_state["integral"] *= 0.9
    elif s[1] and s[2]:
        error = 0
    elif s[1]:
        error = -1
    elif s[2]:
        error = 1
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






## --- Just test code ---
def run_line_follower(mode="pid", debug=False):
    """Main function to run the line follower."""
    print(f"Starting line follower (mode: {mode})...\nPress Ctrl+C to stop.")
    try:
        while True:
            if mode == "pid":
                follow_line_pid()
            elif mode == "basic":
                follow_line_basic()
            else:
                print(f"Unknown mode: {mode}")
            if debug:
                vals = sensor_state
                print(f"Sensors: {' '.join(map(str, vals))} | State: {vals}")
            time.sleep_ms(10)
    except KeyboardInterrupt:
        print("\nStopping line follower...")
        stop_motors()
        print("Stopped.")


if __name__ == "__main__":
    ## test_sensors()
    run_line_follower(mode="basic", debug=True)
