import time
from machine import Pin
from .motor_functions import set_motor_speed, stop_motors
from .constants import ROBOT_CONFIG
from .sensors import sensor_state

pid_state = {
    "last_error": 0.0,
    "integral": 0.0,
    "filtered_derivative": 0.0,
    "turn_end_time": 0
}

def follow_line_basic():
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
    s = sensor_state

    ##Calculate error from sensor readings: -1 = left of line, 0 = on line, 1 = right of line
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

    ##Integral term accumulates error over time, clamped to prevent windup
    pid_state["integral"] = max(-ROBOT_CONFIG.PID_MAX_INTEGRAL,
                                min(ROBOT_CONFIG.PID_MAX_INTEGRAL,
                                    pid_state["integral"] + error))
    ##Derivative term smoothed with exponential filter to reduce noise
    raw_deriv = error - pid_state["last_error"]
    pid_state["filtered_derivative"] = (ROBOT_CONFIG.PID_ALPHA * raw_deriv +
                                        (1 - ROBOT_CONFIG.PID_ALPHA) * pid_state["filtered_derivative"])

    ##Use boosted PID values right after turns for faster line reacquisition
    time_since_turn = time.ticks_diff(time.ticks_ms(), pid_state["turn_end_time"])
    if time_since_turn < ROBOT_CONFIG.POST_TURN_BOOST_DURATION_MS:
        kp = ROBOT_CONFIG.POST_TURN_KP
        kd = ROBOT_CONFIG.POST_TURN_KD
        correction_factor = ROBOT_CONFIG.POST_TURN_CORRECTION_FACTOR
    else:
        kp = ROBOT_CONFIG.PID_KP
        kd = ROBOT_CONFIG.PID_KD
        correction_factor = ROBOT_CONFIG.PID_CORRECTION_FACTOR

    ##PID formula: P + I + D components combined and scaled
    correction = (kp * error + ROBOT_CONFIG.PID_KI * pid_state["integral"] +
                 kd * pid_state["filtered_derivative"]) * correction_factor
    pid_state["last_error"] = error

    ##Apply correction to motor speeds: positive correction = turn right
    base = ROBOT_CONFIG.BASE_SPEED
    left = max(ROBOT_CONFIG.MIN_SPEED, min(255, int(base + correction)))
    right = max(ROBOT_CONFIG.MIN_SPEED, min(255, int(base - correction)))
    set_motor_speed(left, 1, right, 1)






def run_line_follower(mode="pid", debug=False):
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

if __name__ == "__main__":
    run_line_follower(mode="pid", debug=True)
