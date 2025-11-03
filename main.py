
from sw import motor_functions, line_follower, sensors, leds
from sw.sensors import sensor_state
import utime
import _thread
from machine import Pin

utime.sleep(0.2)


def execute_turn(direction):
    print(f"\n[Turn] Executing turn: {direction}")
    for key in line_follower.pid_state:
        if key != "turn_end_time":
            line_follower.pid_state[key] = 0.0
    motor_functions.turn_until_line_on_sensors(direction)
    # Mark the time when turn completes for post-turn boost
    line_follower.pid_state["turn_end_time"] = utime.ticks_ms()
    print(f"[Turn] Turn {direction} complete")

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
            print(f"\n[Navigation] Passed intersection {count}/{target_count}")
            start_time = utime.ticks_ms()

    return count


def follow_line_for_duration(duration_ms):
    """Follow the line for a specified duration in milliseconds"""
    start_time = utime.ticks_ms()
    while utime.ticks_diff(utime.ticks_ms(), start_time) < duration_ms:
        line_follower.follow_line_pid()


def collect_box():
    print("\n[Collect] Approaching box...")
    distance_to_box = sensors.get_tmf8701_distance()
    code = sensors.get_tiny_code().split(",")
    row = code[2]
    print(f"\n[Collect] QR Code: {code}, Target row: {row}")

    while distance_to_box is not None and distance_to_box > 0:
        line_follower.run_line_follower(mode='pid', debug=False)

    print("\n[Collect] Box reached, activating lift mechanism")
    ## Lift up mechanism?
    print("\n[Collect] Turning left 90 degrees")
    motor_functions.turn_left(90)
    execute_turn("left")

    print("\n[Collect] Turning around, heading back...")
    while sensor_state[0] != 1 and sensor_state[3] != 1:
        line_follower.run_line_follower(mode='pid', debug=False)
    motor_functions.stop_motors()
    print("\n[Collect] Collection complete")
    return row




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
    print("\n[Point 3] Fine-tuning turn left 6 degrees")
    motor_functions.turn_left(6)
    print("\n[Point 3] Checking for box at Point 3...")

    distance_to_box = sensors.get_tmf8701_distance()
    print(f"\n[Point 3] Distance to box: {distance_to_box}mm")
    if distance_to_box is not None and distance_to_box < 100:
        print("\n[Point 3] Box detected! Collecting...")
        row = collect_box()
        print(f"\n[Point 3] Box collected, target row: {row}")
        motor_functions.move(speed=255, direction=0, duration_ms=1000)
        execute_turn("left")
        while sensor_state[0] != 1:
            line_follower.follow_line_pid()
        execute_turn("right")

        print(f"\n[Point 3] Navigating to row {row}...")
        follow_line_until_intersections(row, sensor_index=3, debounce_ms=500)
        execute_turn("right")
        motor_functions.move(speed=255, direction=1, duration_ms=500)




    print("\n[Point 3->4] Moving to Point 4...")
    print("\n[Point 3->4] Turning right 40 degrees")
    execute_turn("right")
    follow_line_for_duration(300)
    while sensor_state[0] != 1:
        line_follower.follow_line_pid()


    # Point 4
    print("\n[Point 4] Checking for box at Point 4...")
    execute_turn("left")
    motor_functions.turn_left(5)
    distance_to_box = sensors.get_tmf8701_distance()
    print(f"\n[Point 4] Distance to box: {distance_to_box}mm")
    if distance_to_box is not None and distance_to_box < 100:
        print("\n[Point 4] Box detected! Collecting...")
        collect_box()

    # If no boxes on the left, check right side
    print("\n[Navigation] No boxes on left, checking right side...")
    execute_turn("left")
    follow_line_for_duration(500)
    follow_line_until_intersections(3, sensor_index=3, debounce_ms=300)





    # Point 5
    print("\n[Point 5] Checking for box at Point 5...")
    execute_turn("right")
    motor_functions.turn_right(5)
    distance_to_box = sensors.get_tmf8701_distance()
    print(f"\n[Point 5] Distance to box: {distance_to_box}mm")
    if distance_to_box is not None and distance_to_box < 200:
        print("\n[Point 5] Box detected! Collecting...")
        collect_box()

    print("\n[Point 5->6] Moving to Point 6...")
    execute_turn("left")
    follow_line_for_duration(300)
    while sensor_state[3] != 1:
        line_follower.follow_line_pid()

    # Point 6
    print("\n[Point 6] Checking for box at Point 6...")
    execute_turn("right")
    distance_to_box = sensors.get_tmf8701_distance()
    print(f"\n[Point 6] Distance to box: {distance_to_box}mm")
    if distance_to_box is not None and distance_to_box < 200:
        print("\n[Point 6] Box detected! Collecting...")
        collect_box()

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

    # Start the sensor update thread
    _thread.start_new_thread(sensors.sensor_update_thread, ())
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