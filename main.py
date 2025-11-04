
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
        else:  # scan_direction == "right"
            motor_functions.turn_right(initial_turn_angle)
            for i in range(scan_steps):
                utime.sleep_ms(400)
                motor_functions.turn_right(i)
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

    # Point 2
    follow_line_until_intersections(1, sensor_index=0, debounce_ms=200)
    leds.turn_on_flashing_led()

    # Point 3
    execute_turn("left")
    print("\n[Point 3] Checking for box at Point 3...")

    # Scan for QR code and collect if found
    result = collect_box(scan_for_qr=True, initial_turn_angle=3, scan_steps=5)

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

        elif rack == "Rack B":
            execute_turn("right")
            follow_line_for_duration(500)
            follow_line_until_intersections(3, sensor_index=3, debounce_ms=200)
            execute_turn("left")
            print(f"\n[Point 3] Navigating to row {row}...")
            follow_line_until_intersections(row, sensor_index=0, debounce_ms=500)
            execute_turn("left")
            ##LINEAR ACTUATOR
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
    result = collect_box(scan_for_qr=True, initial_turn_angle=3, scan_steps=5)

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
    else:
        # No QR detected, turn 30 degrees left before execute_turn
        print("[Point 4] No QR detected, turning 30 degrees left before execute_turn")
        motor_functions.turn_left(30)


















## points 5 and 6 still need to be tested at the moment, should be mirrored!


    print("\n[Navigation] No boxes on left, checking right side...")
    execute_turn("left")
    follow_line_for_duration(1000)
    follow_line_until_intersections(3, sensor_index=3, debounce_ms=300)


    # Point 5
    print("\n[Point 5] Checking for box at Point 5...")
    execute_turn("right")

    # Scan for QR code and collect if found
    result = collect_box(scan_for_qr=True, initial_turn_angle=3, scan_steps=7, scan_direction="right")

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

    result = collect_box(scan_for_qr=True, initial_turn_angle=3, scan_steps=7, scan_direction="right")

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
                main()

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
