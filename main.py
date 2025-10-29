"""Cargo delivery robot - main navigation and task execution."""
from sw import motor_functions, line_follower, map, sensors
from sw.linear_actuator import Actuator
from sw.constants import ROBOT_CONFIG
from utime import sleep

## Intersection detection states
TURNING_STATES = [[1,1,1,1], [1,0,0,1], [1,1,1,0], [0,1,1,1], [1,0,0,0], [0,0,0,1]]

## Robot state
intersection_count = 0
current_side = None
has_cargo = False
actuator = Actuator(dirPin=0, PWMPin=1)

def navigate_to_main_line():
    """Move from start position to main line and turn left."""
    print("Finding main line...")
    motor_functions.move_forward(speed=255)

    ## Wait for outer sensors to detect first line
    while True:
        state = line_follower.read_all_sensors()
        if state[0] == 1 and state[3] == 1:
            print("Main line reached")
            break

    ## Follow line until turn point (second intersection)
    while True:
        line_follower.run_line_follower(mode='pid', debug=False)
        state = line_follower.read_all_sensors()
        if state[0] == 1 and state[3] == 1:
            break

    print("Turning left onto main path")
    motor_functions.turn_until_line_on_sensors(line_follower.read_all_sensors, turn_left=True)
    clear_intersection()

def follow_to_next_intersection():
    """Follow line until next intersection is detected."""
    extended_loss_count = 0

    while True:
        state = line_follower.read_all_sensors()

        ## Check for intersection
        if state in TURNING_STATES:
            return state

        ## Monitor for extended line loss
        if not any(state):
            extended_loss_count += 1
            if extended_loss_count > ROBOT_CONFIG.LINE_LOSS_THRESHOLD:
                print(f"Extended line loss detected (count: {extended_loss_count})")
                if not recover_line():
                    ## Critical failure - stop robot
                    motor_functions.stop_motors()
                    raise Exception("CRITICAL: Unable to recover line")
                extended_loss_count = 0
        else:
            extended_loss_count = 0

        line_follower.run_line_follower(mode='pid', debug=False)

def clear_intersection():
    """Move forward to clear the intersection area."""
    for _ in range(15):
        line_follower.run_line_follower(mode='pid')

def reset_pid():
    """Reset PID state to prevent overshoot."""
    for key in line_follower.pid_state:
        line_follower.pid_state[key] = 0.0

def recover_line():
    """Emergency line recovery when completely lost."""
    print("WARNING: Line lost! Executing recovery...")
    reset_pid()

    ## Try backing up slightly and searching
    motor_functions.move_backward(speed=150, duration_ms=ROBOT_CONFIG.RECOVERY_BACKUP_MS)
    sleep(0.1)

    ## Search left then right
    for direction in ['left', 'right']:
        motor_functions.stop_motors()
        sleep(0.05)

        ## Small search turn
        speed = ROBOT_CONFIG.RECOVERY_SEARCH_SPEED
        if direction == 'left':
            motor_functions.set_motor_speed(speed, 0, speed, 1)
        else:
            motor_functions.set_motor_speed(speed, 1, speed, 0)

        ## Check for line during search
        for _ in range(ROBOT_CONFIG.RECOVERY_SEARCH_STEPS):
            state = line_follower.read_all_sensors()
            if any(state):
                print(f"Line recovered! Sensor state: {state}")
                motor_functions.stop_motors()
                reset_pid()
                return True
            sleep(0.02)

    motor_functions.stop_motors()
    print("ERROR: Line recovery failed")
    return False

def turn_to_spot(turn_left):
    """Turn off main line to cargo/bay spot."""
    motor_functions.move_forward(speed=255, duration_ms=150)
    sleep(0.2)
    reset_pid()
    motor_functions.turn_until_line_on_sensors(line_follower.read_all_sensors, turn_left=turn_left)
    clear_intersection()

def check_and_pickup_cargo():
    """Check for cargo at current location and pick it up if present."""
    global has_cargo

    if sensors.check_box_present():
        print("Box detected! Scanning QR code...")
        qr_code = sensors.scan_qr_code()

        if qr_code:
            print(f"QR Code: {qr_code}")
            rack, level, bay = map.parse_qr_code(qr_code)

            if rack and level and bay:
                print(f"Destination: Rack {rack}, {level}, Bay {bay}")
                ## Lift box with actuator
                actuator.set(dir=1, speed=100)
                sleep(1)

                ## Verify box has been picked up using TMF8701
                if sensors.verify_box_picked_up():
                    print("Box pickup confirmed")
                    has_cargo = True
                    return rack, level, bay
                else:
                    print("Warning: Box pickup not confirmed")
                    has_cargo = False

    print("No box found")
    return None, None, None

def navigate_to_bay(target_intersection, target_side):
    """Navigate from current position to target bay intersection."""
    global intersection_count, current_side

    ## Return to main line
    motor_functions.turn_until_line_on_sensors(line_follower.read_all_sensors, turn_left=False)
    clear_intersection()

    ## Switch to Rack B if needed
    if current_side != target_side and target_side == map.RACK_B_SIDE:
        print(f"Switching from {current_side} to {target_side}")
        while intersection_count < map.CARGO_SPOT_1_RIGHT - 1:
            follow_to_next_intersection()
            intersection_count += 1
            print(f"Passed intersection {intersection_count}")
            clear_intersection()
        current_side = target_side

    ## Navigate to target bay
    while intersection_count < target_intersection:
        follow_to_next_intersection()
        intersection_count += 1
        print(f"Passed intersection {intersection_count}")
        clear_intersection()

def deliver_cargo(rack, level, target_bay):
    """Navigate to target bay and deliver cargo."""
    global has_cargo

    target_intersection = map.get_bay_intersection(rack, target_bay)
    if target_intersection is None:
        print(f"Error: Invalid rack {rack} or bay {target_bay}")
        return

    print(f"Navigating to Rack {rack}, Bay {target_bay} (intersection {target_intersection})")
    navigate_to_bay(target_intersection, map.get_side_from_rack(rack))

    ## Turn into bay
    print(f"Arrived at Rack {rack} Bay {target_bay}. Turning in...")
    turn_to_spot(current_side == map.RACK_A_SIDE)

    ## Use precision sensor to position at correct distance
    distance = sensors.get_bay_distance()
    print(f"Bay distance: {distance}mm")

    ## Deliver cargo at appropriate height
    if level == 'Upper':
        actuator.set(dir=1, speed=100)
        sleep(2)

    actuator.set(dir=0, speed=100)
    sleep(1)
    has_cargo = False
    print("Cargo delivered!")

    ## Return to main line
    print("Returning to main line...")
    motor_functions.turn_until_line_on_sensors(line_follower.read_all_sensors, turn_left=False)
    clear_intersection()

def main():
    """Main cargo delivery loop."""
    global intersection_count, current_side

    sleep(2)
    print("Starting cargo delivery robot...")

    navigate_to_main_line()
    current_side = map.RACK_A_SIDE
    intersection_count = 0

    while True:
        follow_to_next_intersection()
        intersection_count += 1
        location = map.INTERSECTION_MAP.get(intersection_count, f"intersection_{intersection_count}")
        print(f"\nReached: {location} (#{intersection_count})")

        ## Check if this is a cargo spot
        if 'cargo' in location:
            turn_to_spot(current_side == map.RACK_A_SIDE)
            rack, level, bay = check_and_pickup_cargo()

            if rack and level and bay:
                deliver_cargo(rack, level, bay)
                ## After delivery, already returned to main line
            else:
                ## No cargo, return to main line
                motor_functions.turn_until_line_on_sensors(line_follower.read_all_sensors, turn_left=False)
                clear_intersection()

        ## Switch sides at the end of Rack A
        elif intersection_count == map.RACK_A_BAY_6:
            print("Reached end of Rack A, continuing to Rack B")
            current_side = map.RACK_B_SIDE
            clear_intersection()

        ## Continue past non-cargo intersections
        else:
            clear_intersection()

if __name__ == "__main__":
    main()

