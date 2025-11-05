
from sw import motor_functions, line_follower, sensors, leds, linear_actuator
from sw.sensors import sensor_state
import utime
import _thread
from machine import Pin

utime.sleep(0.2)


def execute_turn(direction):
    print(f"\n\n[Turn] Executing turn: {direction}")
    for key in line_follower.pid_state:
        if key != "turn_end_time":
            line_follower.pid_state[key] = 0.0
    motor_functions.turn_until_line_on_sensors(direction)
    # Mark the time when turn completes for post-turn boost
    line_follower.pid_state["turn_end_time"] = utime.ticks_ms()
    print(f"\n[Turn] Turn {direction} complete")

    # Post-turn recovery: only if one center sensor is off after turn
    utime.sleep_ms(20)  # Brief settle time

    # Check if we need recovery (one center sensor off, one on = bad angle)
    if sensor_state[1] == 1 and sensor_state[2] == 0:
        # Right sensor off, turn right sharply
        print("[Recovery] Right sensor off line, correcting right...")
        recovery_start = utime.ticks_ms()
        while utime.ticks_diff(utime.ticks_ms(), recovery_start) < 200:
            if sensor_state[1] == 1 and sensor_state[2] == 1:
                print("[Recovery] Line recovered")
                break
            motor_functions.set_motor_speed(255, 1, 255, 0)  # Sharp right turn
            utime.sleep_ms(10)
        motor_functions.stop_motors()
    elif sensor_state[1] == 0 and sensor_state[2] == 1:
        # Left sensor off, turn left sharply
        print("[Recovery] Left sensor off line, correcting left...")
        recovery_start = utime.ticks_ms()
        while utime.ticks_diff(utime.ticks_ms(), recovery_start) < 200:
            if sensor_state[1] == 1 and sensor_state[2] == 1:
                print("[Recovery] Line recovered")
                break
            motor_functions.set_motor_speed(255, 0, 255, 1)  # Sharp left turn
            utime.sleep_ms(10)
        motor_functions.stop_motors()
    # If both center sensors are on (or both off), trust the PID boost to handle it


def follow_line_until_intersections(target_count, sensor_index=0, debounce_ms=200):
    count = 0
    start_time = utime.ticks_ms()

    while count != target_count:
        line_follower.follow_line_pid()
        if sensor_state[sensor_index] == 1 and utime.ticks_ms() - start_time > debounce_ms:
            count += 1
            print(f"\n\n[Navigation] Passed intersection {count}/{target_count}")
            start_time = utime.ticks_ms()

    return count


def follow_line_until_distance(target_distance_mm):
    """Follow the line until the distance sensor reads less than the target distance."""
    distance_to_box = sensors.get_tmf8701_distance()
    # Add a small delay to allow the sensor to get a stable reading
    utime.sleep_ms(50)
    distance_to_box = sensors.get_tmf8701_distance()

    while distance_to_box is None or distance_to_box > target_distance_mm:
        line_follower.follow_line_pid()
        distance_to_box = sensors.get_tmf8701_distance()
        # Optional: add a small delay to avoid spamming the sensor)

    motor_functions.stop_motors()
    print(f"\n\n[Navigation] Reached target distance: {distance_to_box}mm")


def follow_line_for_duration(duration_ms):
    """Follow the line for a specified duration in milliseconds"""
    start_time = utime.ticks_ms()
    while utime.ticks_diff(utime.ticks_ms(), start_time) < duration_ms:
        line_follower.follow_line_pid()


def return_to_start_from_rack_a(row):
    """Return to start position from Rack A after dropping off a box"""
    follow_line_for_duration(200)
    follow_line_until_intersections(row, sensor_index=0, debounce_ms=500)
    execute_turn("left")
    follow_line_for_duration(400)
    follow_line_until_intersections(2, sensor_index=3, debounce_ms=500)
    ###POINT OF MOVING TO 5 and 6

def return_to_start_from_rack_b(row):
    """Return to start position from Rack B after dropping off a box"""
    follow_line_for_duration(200)
    follow_line_until_intersections(row, sensor_index=3, debounce_ms=500)
    execute_turn("right")
    follow_line_for_duration(400)
    follow_line_until_intersections(2, sensor_index=3, debounce_ms=500)
    ###POINT OF MOVING TO 3 and 4


def collect_box(scan_for_qr=False, initial_turn_angle=0, scan_steps=7, scan_direction="left"):
    """
    Collect a box, optionally scanning for QR code first.

    Args:
        scan_for_qr: If True, scans incrementally to find QR code
        initial_turn_angle: Initial turn angle before scanning
        scan_steps: Number of scanning steps
        scan_direction: Direction to scan - "left" or "right"

    Returns:
        row: The target row from QR code, or None if no box found during scan
    """
    print("\n\n[Collect] Starting collection...")

    # Optional QR code scanning
    code = None
    if scan_for_qr:
        print(f"[Collect] Scanning for QR code ({scan_direction})...")
        sensors.enable_qr_scanning()  # Enable QR scanning only when needed
        utime.sleep_ms(100)  # Brief delay to allow first scan

        if scan_direction == "left":
            motor_functions.turn_left(initial_turn_angle)
            for i in range(scan_steps):
                utime.sleep_ms(400)
                motor_functions.turn_left(i)
                code = sensors.get_tiny_code()
                if code is not None and code != "No QR code detected":
                    print(f"[Collect] QR code found: {code}")
                    break
        elif scan_direction == "right":  # scan_direction == "right"
            motor_functions.turn_right(initial_turn_angle)
            for i in range(scan_steps):
                utime.sleep_ms(400)
                motor_functions.turn_right(i)
                code = sensors.get_tiny_code()
                if code is not None and code != "No QR code detected":
                    print(f"[Collect] QR code found: {code}")
                    break
        elif scan_direction == "both":
            for i in range(scan_steps):
                utime.sleep_ms(400)
                motor_functions.turn_left(i)
                code = sensors.get_tiny_code()
                if code is not None and code != "No QR code detected":
                    print(f"[Collect] QR code found: {code}")
                    break
            for i in range(scan_steps*2):
                utime.sleep_ms(400)
                motor_functions.turn_right(i)
                code = sensors.get_tiny_code()
                if code is not None and code != "No QR code detected":
                    print(f"[Collect] QR code found: {code}")
                    break
            for i in range(scan_steps):
                utime.sleep_ms(400)
                motor_functions.turn_left(i)
                code = sensors.get_tiny_code()
                if code is not None and code != "No QR code detected":
                    print(f"[Collect] QR code found: {code}")
                    break



        sensors.disable_qr_scanning()  # Disable QR scanning after we're done

        # If no QR code found during scan, return None
        if code is None or code == "No QR code detected":
            print("[Collect] No QR code detected during scan, aborting collection")
            return None
    else:
        sensors.enable_qr_scanning()
        utime.sleep_ms(200)  # Allow time for one scan
        code = sensors.get_tiny_code()
        sensors.disable_qr_scanning()

    # Parse QR code for row and rack information
    rack = None
    if code and code != "No QR code detected":
        code_parts = code.split(",")
        rack = code_parts[0].strip() if len(code_parts) > 0 else "Rack A"
        row = int(code_parts[2].strip()) if len(code_parts) > 2 else 1
    else:
        rack = "Rack A"
        row = 1  # Default row if no QR code
    print(f"\n[Collect] QR Code: {code}, Rack: {rack}, Target row: {row}")

    # Approach the box using VL53L0X sensor (more reliable for box detection)
    print("[Collect] Approaching box...")
    distance_to_box = sensors.get_vl53l0x_distance()
    target_distance = 100  # Stop when directly in front of the box (<50mm)

    while distance_to_box is not None and distance_to_box > target_distance:
        line_follower.follow_line_pid()
        distance_to_box = sensors.get_vl53l0x_distance()
        utime.sleep_ms(10)  # Small delay to avoid sensor spam

    print(f"\n[Collect] Box reached at {distance_to_box}mm, moving forward to pick up box...")
    motor_functions.move(speed=255, direction=1, duration_ms=500)

    print("\n[Collect] Activating lift mechanism - lifting for 1000ms")
    linear_actuator.actuator.move(linear_actuator.RETRACT_DIRECTION, 100)  # Lift at full speed
    utime.sleep_ms(1000)
    linear_actuator.actuator.stop()
    print("\n[Collect] Collection complete")
    return (row, rack)



def main():
    print("\n=== Starting main routine ===")
    utime.sleep(0.5)

    # Point 1
    follow_line_until_intersections(2, sensor_index=0, debounce_ms=200)
    execute_turn("left")

    ############### Point 2- MOVING TO 3 and 4 for movebackto start functions- line
    follow_line_until_intersections(1, sensor_index=0, debounce_ms=200)
    leds.turn_on_flashing_led()


    # Point 3
    execute_turn("left")
    print("\n[Point 3] Checking for box at Point 3...")

    # Scan for QR code and collect if found
    result = collect_box(scan_for_qr=True, initial_turn_angle=5, scan_steps=5)

    if result is not None:
        row, rack = result
        print(f"\n[Point 3] Box collected, Rack: {rack}, Target row: {row}")

        # Only navigate to drop-off if it's Rack A
        motor_functions.turn_left(90)
        execute_turn("left")
        follow_line_for_duration(500)
        while sensor_state[0] != 1:
            line_follower.follow_line_pid()
        if rack == "Rack A":
            execute_turn("left")
            follow_line_for_duration(500)
            while sensor_state[0] != 1:
                line_follower.follow_line_pid()
            execute_turn("right")
            print(f"\n[Point 3] Navigating to row {row}...")
            follow_line_until_intersections(row, sensor_index=3, debounce_ms=500)
            execute_turn("right")
            ##LINEAR ACTUATOR
            print("WHOOOO")
            utime.sleep(444)
            # Return to start
            return_to_start_from_rack_a(row)

        elif rack == "Rack B":
            execute_turn("right")
            follow_line_for_duration(500)
            follow_line_until_intersections(3, sensor_index=3, debounce_ms=200)
            execute_turn("left")
            print(f"\n[Point 3] Navigating to row {row}...")
            follow_line_until_intersections(row, sensor_index=0, debounce_ms=500)
            execute_turn("left")
            ##LINEAR ACTUATOR
            # Return to start
            return_to_start_from_rack_b(row)
    else:
        # No QR detected, turn 30 degrees left before execute_turn
        print("[Point 3] No QR detected, turning 30 degrees left before execute_turn")
        motor_functions.turn_right(30)

    row = None
    rack = None




    print("\n[Point 3->4] Moving to Point 4...")
    execute_turn("right")
    follow_line_for_duration(500)
    while sensor_state[0] != 1:
        line_follower.follow_line_pid()

    execute_turn("left")
    result = collect_box(scan_for_qr=True, initial_turn_angle=5, scan_steps=5)

    if result is not None:
        row, rack = result
        print(f"\n[Point 4] Box collected, Rack: {rack}, Target row: {row}")

        # Only navigate to drop-off if it's Rack A
        motor_functions.turn_left(90)
        execute_turn("left")
        ##going back, north on point 4
        if rack == "Rack A":
            follow_line_for_duration(2000)
            print(f"\n[Point 4] Navigating to row {row}...")
            follow_line_until_intersections(row, sensor_index=3, debounce_ms=500)
            execute_turn("right")
            ##LINEAR ACTUATOR
            print("WHOOOO")
            utime.sleep(444)
            # Return to start
            return_to_start_from_rack_a(row)

        elif rack == "Rack B":
            follow_line_for_duration(500)
            while sensor_state[3] != 1:
                line_follower.follow_line_pid()
            execute_turn("right")
            follow_line_for_duration(500)

            follow_line_until_intersections(4, sensor_index=3, debounce_ms=200)
            execute_turn("left")
            print(f"\n[Point 3] Navigating to row {row}...")
            follow_line_until_intersections(row, sensor_index=0, debounce_ms=500)
            execute_turn("left")

            ##LINEAR ACTUATOR
            # Return to start
            return_to_start_from_rack_b(row)
    else:
        # No QR detected, turn 30 degrees left before execute_turn
        print("[Point 4] No QR detected, turning 30 degrees left before execute_turn")
        motor_functions.turn_left(30)
    ## points 5 and 6 still need to be tested at the moment, should be mirrored!
    ################## Point- MOVING TO 5 and 6 for movebackto start functions- line
    print("\n[Navigation] No boxes on left, checking right side...")
    execute_turn("left")
    follow_line_for_duration(1000)
    follow_line_until_intersections(3, sensor_index=3, debounce_ms=300)


    # Point 5
    print("\n[Point 5] Checking for box at Point 5...")
    execute_turn("right")

    # Scan for QR code and collect if found
    result = collect_box(scan_for_qr=True, initial_turn_angle=5, scan_steps=7, scan_direction="right")

    if result is not None:
        row, rack = result
        print(f"\n[Point 5] Box collected, Rack: {rack}, Target row: {row}")

        # Navigate to drop-off (reversed for right side)
        motor_functions.turn_right(90)
        execute_turn("right")
        follow_line_for_duration(500)
        while sensor_state[3] != 1:
            line_follower.follow_line_pid()

        if rack == "Rack A":
            execute_turn("left")
            follow_line_for_duration(500)
            follow_line_until_intersections(3, sensor_index=0, debounce_ms=200)
            execute_turn("right")
            follow_line_for_duration(500)
            print(f"\n[Point 5] Navigating to row {row}...")
            follow_line_until_intersections(row, sensor_index=0, debounce_ms=500)
            execute_turn("left")
            ##LINEAR ACTUATOR
            print("WHOOOO")
            utime.sleep(444)
            # Return to start
            return_to_start_from_rack_a(row)


        elif rack == "Rack B":
            execute_turn("right")
            follow_line_for_duration(500)
            while sensor_state[0] != 1:
                line_follower.follow_line_pid()
            follow_line_for_duration(500)
            print(f"\n[Point 5] Navigating to row {row}...")
            follow_line_until_intersections(row, sensor_index=3, debounce_ms=500)
            execute_turn("left")
            ##LINEAR ACTUATOR
            # Return to start
            return_to_start_from_rack_b(row)
    else:
        # No QR detected, turn 30 degrees right before execute_turn
        print("[Point 5] No QR detected, turning 30 degrees right before execute_turn")
        motor_functions.turn_left(30)




    row = None
    rack = None

    print("\n[Point 5->6] Moving to Point 6...")
    execute_turn("left")
    follow_line_for_duration(500)
    while sensor_state[3] != 1:
        line_follower.follow_line_pid()

    # Point 6
    print("\n[Point 6] Checking for box at Point 6...")
    execute_turn("right")

    result = collect_box(scan_for_qr=True, initial_turn_angle=5, scan_steps=7, scan_direction="right")

    if result is not None:
        row, rack = result
        print(f"\n[Point 6] Box collected, Rack: {rack}, Target row: {row}")

        # Navigate to drop-off (reversed for right side)
        motor_functions.turn_left(90)
        execute_turn("left")
        ##going back, north on point 6
        if rack == "Rack B":
            follow_line_for_duration(2000)
            print(f"\n[Point 6] Navigating to row {row}...")
            follow_line_until_intersections(row, sensor_index=0, debounce_ms=500)
            execute_turn("left")
            ##LINEAR ACTUATOR
            print("WHOOOO")
            utime.sleep(444)
            # Return to start
            return_to_start_from_rack_b(row)

        elif rack == "Rack A":
            follow_line_for_duration(500)
            while sensor_state[0] != 1:
                line_follower.follow_line_pid()
            execute_turn("left")
            follow_line_for_duration(500)

            follow_line_until_intersections(4, sensor_index=0, debounce_ms=200)
            execute_turn("right")
            follow_line_for_duration(500)
            print(f"\n[Point 6] Navigating to row {row}...")
            follow_line_until_intersections(row, sensor_index=3, debounce_ms=500)
            execute_turn("right")
            ##LINEAR ACTUATOR
            # Return to start
            return_to_start_from_rack_a(row)
    else:
        # No QR detected, turn 30 degrees right before execute_turn
        print("[Point 6] No QR detected, turning 30 degrees right before execute_turn")
        motor_functions.turn_right(30)

    execute_turn("right")
    follow_line_for_duration(500)

    # No boxes found, return to start
    print("\n[Return] No boxes found, returning to start...")
    follow_line_until_intersections(2, sensor_index=0, debounce_ms=300)
    execute_turn("left")
    print("\n=== Routine complete ===")


actuator = linear_actuator.Actuator(dir_pin=0, pwm_pin=1)

#---------------------------------------KibAndChe-------------------------------------
#---------------------------------------KibAndChe-------------------------------------




print("hellow orld")




actuator = linear_actuator.Actuator(dir_pin=0, pwm_pin=1)








sensorArr = [0, 0, 0, 0]
rollList = ["", "", "", "", "", "", ""]




map = ["A1", "A2", "A3", "A4", "A5", "A6", "LB1T", "LB2", "S8", "LB3", "LB4T",
       "B1", "B2", "B3", "B4", "B5", "B6"]




turnsAnti = [1, 1, 1, 1, 1, 1, 0, -1, -1, -1, -1, 1, 1, 1, 1, 1, 1]
turnsClock = [-1, -1, -1, -1, -1, -1, 1, 1, 1, 1, 0, -1, -1, -1, -1, -1, -1]
turns = [turnsAnti, turnsClock]




pointer = 8
dest = 9
dir = 0  # 0 anticlockwise, 1 clockwise
upper = False
loadingBays = [6, 7, 9, 10]
loadingBaysList = [6, 7, 10]
pathViewToggle = False




def decide_movement(sensorArr):
    if sensorArr == [1, 1, 1, 1]:
        return "S"
    elif sensorArr == [1, 0, 0, 1]:
        return "LR"
    elif sensorArr == [1, 1, 1, 0]:
        return "SL"
    elif sensorArr == [0, 1, 1, 1]:
        return "SR"
    elif sensorArr == [0, 1, 1, 0]:
        return "S"
    elif sensorArr == [0, 0, 1, 0]:
        return "S"
    elif sensorArr == [0, 1, 0, 0]:
        return "S"
    elif sensorArr == [1, 0, 0, 0]:
        return "L"
    elif sensorArr == [0, 0, 0, 1]:
        return "R"
    elif sensorArr == [0, 0, 0, 0]:
        return "party"
    elif sensorArr == [1, 1, 0, 0]:
        return "AL"
    elif sensorArr == [0, 0, 1, 1]:
        return "AR"
    elif sensorArr == [1, 0, 1, 0]:
        return "AR"
    elif sensorArr == [0, 1, 0, 1]:
        return "AL"
    elif sensorArr == [1, 0, 1, 1]:
        return "AR"
    elif sensorArr == [1, 1, 0, 1]:
        return "AL"
    else:
        return "party"




def nav():
    global pointer, pathViewToggle, dir, dest, loadingBaysList, loadingBays
    TurnToggles = [True,True]
    # 6,10 toggles
    pointer = 8
    dest = 9
    dir = 0  # 0 anticlockwise, 1 clockwise
    upper = False
    loadingBays = [6, 7, 9, 10]
    loadingBaysList = [6, 7, 10]
    pathViewToggle = False


    #---------Starting and getting to main line-----------
    # Point 1
    follow_line_until_intersections(2, sensor_index=0, debounce_ms=200)
    execute_turn("right")
    #---------------------------
    def ScanForCode():
        scan_steps = 7
        sensors.enable_qr_scanning()  # Enable QR scanning only when needed
        utime.sleep_ms(100)  # Brief delay to allow first scan

        for i in range(scan_steps):
            utime.sleep_ms(400)
            motor_functions.turn_left(i)
            code = sensors.get_tiny_code()
            if code is not None and code != "No QR code detected":
                print(f"[Collect] QR code found: {code}")
                break
        for i in range(scan_steps*2):
            utime.sleep_ms(400)
            motor_functions.turn_right(i)
            code = sensors.get_tiny_code()
            if code is not None and code != "No QR code detected":
                print(f"[Collect] QR code found: {code}")
                break
        for i in range(scan_steps):
            utime.sleep_ms(400)
            motor_functions.turn_left(i)
            code = sensors.get_tiny_code()
            if code is not None and code != "No QR code detected":
                print(f"[Collect] QR code found: {code}")
                break

        return code

    def loadUp():
        follow_line_for_duration(200)
        code = None
        for i in range(50):
            code = ScanForCode()
            follow_line_for_duration(50)
            if code is not None and code != "No QR code detected":
                break

        if code is not None and code != "No QR code detected":
            #Splitting up code into constituent parts and setting upper to True if necessary
            parts = code.split(",")  # ["Rack A", " Upper", " 5"]
            row, rack = int(parts[2].strip()), parts[0].strip()
            upper = True if "Upper" in parts[1] else False

            #Move forwards until forklift definitely in loading bay area hopefully it its pusshed on
            while sensor_state[0] != 1 and sensor_state[3] != 1:
                line_follower.follow_line_pid()
            actuator.move(1,100)
            utime.sleep_ms(300)

            #Turning 180 from loading bay area
            motor_functions.turn_left(90)
            execute_turn("left")

            #Attempting to leave laoding bay area
            follow_line_for_duration(500)

            #Going back to intersection
            while sensor_state[0] != 1 and sensor_state[3] != 1:
                line_follower.follow_line_pid()

            rack_map = {
                (1, "Rack A"): 0,
                (2, "Rack A"): 1,
                (3, "Rack A"): 2,
                (4, "Rack A"): 3,
                (5, "Rack A"): 4,
                (6, "Rack A"): 5,
                (1, "Rack B"): 11,
                (2, "Rack B"): 12,
                (3, "Rack B"): 13,
                (4, "Rack B"): 14,
                (5, "Rack B"): 15,
                (6, "Rack B"): 16,
            }
            dest = rack_map[int(row),rack]
        else:
            motor_functions.turn_left(90)
            execute_turn("left")
            loadingBayCheck()
        while sensor_state[0] != 1 and sensor_state[3] != 1:
                line_follower.follow_line_pid()
        #Note this returns the bot facing away from the bay ( opposite to orientation it began the sequence )

    def unloadDown():
        if upper:
            unload_upper()
        else:
            unload_lower()

    def loadingBayCheck():
        global loadingBaysList, dest
        if pointer <= 7:
            dest = loadingBaysList.pop(0)
        else:
            dest = loadingBaysList.pop(-1)
        if  loadingBaysList == []:
            loadingBaysList = loadingBays.copy()
    #Detects the "closest" losfing bay and sets dest to it, removes from list so not to go again
    # resets list if it leaves it empty
    while True:

        print(pointer, dest, dir)
        line_follower.follow_line_pid()
        rollListUpdate()
        all_equal = len(set(rollList[-3:])) == 1
        lastElm = rollList[-1]
        if all_equal and lastElm != "S" and pathViewToggle:
            pointer += [1, -1][dir]
            pathViewToggle = False
        elif all_equal and lastElm == "S":
            pathViewToggle = True


        if pointer == dest:
            print("I AM DEST")
            if turns[dir][pointer] != 0:
                execute_turn(["right", "left"][(turns[dir][pointer] + 1) // 2])
            if pointer in (9, 10, 7, 6):
                #Loading bay
                loadUp()
                dir = 0 if dest > pointer else 1
                if turns[dir][pointer] != 0:
                    execute_turn(["right", "left"][(turns[dir][pointer] + 1) // 2])
                elif turns[dir][pointer] == 0:
                    execute_turn(["right", "left"][(turns[1-dir][pointer]*-1 + 1) // 2])
                    execute_turn(["right", "left"][(turns[1-dir][pointer]*-1 + 1) // 2])
            elif pointer in (0,1,2,3,4,5,11,12,13,14,15,16):
                actuator.move(0,100)
                utime.sleep_ms(300)
                #Storage bay
                #Store box in upper or lower rack based on 'upper' variable
                unloadDown()
                loadingBayCheck()
                dir = 0 if dest > pointer else 1
                execute_turn(["right", "left"][(turns[dir][pointer]*-1 + 1) // 2])
            elif pointer == 8:
                #Station
                #move back into starting box
                pass
        if pointer == 6 and TurnToggles[0]:
            execute_turn(["left", "right"][dir])
            print("I AM TUNRNED6")
            TurnToggles[0] = False
        elif pointer == 10 and TurnToggles[1]:
            execute_turn(["left", "right"][dir])
            print("I AM TUNRNED10")
            TurnToggles[1] = False
        if pointer != 6 and pointer != 10:
            TurnToggles = [True, True]


def rollListUpdate():
    global rollList
    input_val = decide_movement(sensor_state)
    rollList.append(input_val)
    rollList.pop(0)


#---------------------------Unloading
def line_to_rack_movement():
    follow_line_for_duration(400)
    motor_functions.stop_motors()

def lower_level_height():
    actuator = linear_actuator.Actuator(dir_pin=0, pwm_pin=1)
    print("unloading")
    actuator.move(1, 100)  # Lift at full speed
    utime.sleep_ms(1600)
    print("done smth")
    actuator.stop()
    #forklift goes from neutral to L1 height
    # We have no sensors so just move forward for a set time

def upper_level_height():
    actuator = linear_actuator.Actuator(dir_pin=0, pwm_pin=1)
    actuator.move(1, 100)  # Lift at full speed
    utime.sleep_ms(5000)
    actuator.stop()
    #forklift goes from L1 height to L2 height

def return_from_lower_rack_to_mid():
    actuator = linear_actuator.Actuator(dir_pin=0, pwm_pin=1)
    actuator.move(0, 100)  # Lift at full speed
    utime.sleep_ms(1400)
    actuator.stop()
    #forklift goes from L1 height to neutral

actuator = linear_actuator.Actuator(dir_pin=0, pwm_pin=1)

def unload_lower():
    line_to_rack_movement()
    lower_level_height()
    follow_line_for_duration(450)
    motor_functions.stop_motors()
    actuator.move(0,100)
    utime.sleep_ms(300)
    print("before actuator stop")
    actuator.stop()
    print("after actuator stop")
    motor_functions.move(speed=200, direction=0, duration_ms=700)
    actuator.move(0,100)
    utime.sleep_ms(1200)
    actuator.stop()
    # execute_turn("right")
    # execute_turn("right")
    # follow_line_for_duration(200)
    motor_functions.stop_motors()

def unload_upper():
    line_to_rack_movement()
    upper_level_height()
    follow_line_for_duration(450)
    motor_functions.stop_motors()
    actuator.move(0,100)
    utime.sleep_ms(300)
    print("before actuator stop")
    actuator.stop()
    print("after actuator stop")
    motor_functions.move(speed=200, direction=0, duration_ms=700)
    actuator.move(0,100)
    utime.sleep_ms(6000)
    actuator.stop()
    # execute_turn("right")
    # execute_turn("right")
    # follow_line_for_duration(200)
    motor_functions.stop_motors()
#---------------------------------------End Unloading------------------------------------




def fix_linear_actuator():
    actuator.move(0,30) ####1 to go upwards with the crane
    print("heleoeoel")
    utime.sleep_ms(10000)



if __name__ == "__main__":
    utime.sleep(0.4)
    print("Hello from main.py!")
    leds.turn_off_flashing_led()
    print("Flashing stopped")

    button_pin = 28
    button = Pin(button_pin, Pin.IN, Pin.PULL_DOWN)

    # Start the unified sensor thread (I2C sensors + line sensors)
    try:
        _thread.start_new_thread(sensors.run_all_sensors, ())
        print("Sensor thread started (I2C + line sensors)")
    except OSError as e:
        print(f"Sensor thread already running: {e}")

    utime.sleep(2)

    # Safety parameters - set MAX_RUNTIME_MS to limit run time (None = run indefinitely)
    MAX_RUNTIME_MS = 20000
    start_ms = utime.ticks_ms()

    try:
        while True:
            if button.value() == 1:
              while True:
                nav()

            # Enforce optional runtime limit
            if MAX_RUNTIME_MS is not None and utime.ticks_diff(utime.ticks_ms(), start_ms) > MAX_RUNTIME_MS:
                print("Max runtime reached, exiting main loop.")
                break
    except KeyboardInterrupt:
        print("Interrupted by user (KeyboardInterrupt). Stopping.")
        leds.turn_off_flashing_led()
    finally:
        # Ensure motors are stopped when exiting
        try:
            motor_functions.stop_motors()
        except Exception:
            pass
