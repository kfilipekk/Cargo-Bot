print("Hello from main.py!")

from sw import motor_functions
from sw import line_follower
from utime import sleep

# Turning states for intersection detection
turning_states = [
    [1, 1, 1, 1],  ## All sensors on line (4-way intersection)
    [1, 0, 0, 1],  ## Outer sensors on line
    [1, 1, 1, 0],  ## Left three sensors on line (left turn available)
    [0, 1, 1, 1],  ## Right three sensors on line (right turn available)
    [1, 0, 0, 0],  ## Only leftmost sensor on line
    [0, 0, 0, 1],  ## Only rightmost sensor on line
]


    ##go on line, go straight until it reaches the  14 sensors.
# If its a left or right turn, turn and then turn left/right.
#  If they're both on continue going straight






def reset_pid_state():
    """Reset PID controller state."""
    for key in line_follower.pid_state:
        line_follower.pid_state[key] = 0.0


def move_past_intersection():
    """Move forward to clear the intersection."""
    motor_functions.move_forward(speed=255, duration_ms=150)
    sleep(0.2)

def execute_turn(direction):
    """
    Execute a turn at an intersection.
    Args:
        direction: "left" or "right"
    """
    print(f"Turning {direction.upper()}")
    move_past_intersection()
    reset_pid_state()

    ## Use the new function to turn until both sensors 8 and 9 see the line
    if direction == "left":
        motor_functions.turn_until_line_on_sensors(line_follower.read_all_sensors, turn_left=True)
    else:
        motor_functions.turn_until_line_on_sensors(line_follower.read_all_sensors, turn_left=False)

    ## Continue line following after alignment
    print("Line detected on both center sensors. Continuing line following...")


def determine_turn_direction(sensor_state):
    """
    Determine turn direction based on sensor pattern.

    Args:
        sensor_state: List of 4 sensor values

    Returns:
        "left", "right", or None if not an intersection
    """
    if sensor_state[0] == 1: ## [1, x, x, 0]
        return "left"
    elif sensor_state[3] == 1: ## [0, x, x, 1]
        return "right"
    elif sensor_state in turning_states:
        return "left"  ## Default for other intersection types
    return None


def handle_intersection(turn_count, current_state):
    """
    Handle detection and execution of a turn at an intersection.

    Args:
        turn_count: Current count of intersections encountered
        current_state: Current sensor state list

    Returns:
        None
    """
    print(f"\n{'='*50}")
    print(f"INTERSECTION {turn_count} DETECTED!")
    print(f"Sensor state: {current_state}")
    print(f"{'='*50}\n")

    direction = determine_turn_direction(current_state)
    if direction:
        execute_turn(direction)


def clear_intersection():
    """Move forward briefly to clear the intersection area."""
    print("Clearing intersection...")
    for _ in range(15):
        line_follower.run_line_follower(mode='pid')



def move_from_start():



if __name__ == "__main__":
    sleep(2)

    print("Starting navigation...")
    print(f"Looking for these turning states: {turning_states}")

    current_state = line_follower.read_all_sensors()
    print(f"Initial state: {current_state}")

    turn_count = 0

    ## Main loop: follow line and handle turns
    while True:
        ## Follow line until turning state detected
        while current_state not in turning_states:
            line_follower.run_line_follower(mode='pid', debug=False)
            current_state = line_follower.read_all_sensors()

        ## Handle intersection
        turn_count += 1
        handle_intersection(turn_count, current_state)
        clear_intersection()

        ## Update state to continue
        current_state = line_follower.read_all_sensors()
        print(f"Continuing... current state: {current_state}\n")
