
from sw import motor_functions, line_follower, sensors, leds, linear_actuator
from sw.sensors import sensor_state
import utime
import _thread
from machine import Pin

utime.sleep(0.2)

boxes_collected = 0

def execute_turn(direction):
    print(f"\n\n[Turn] Executing turn: {direction}")

    for key in line_follower.pid_state:
        if key != "turn_end_time":
            line_follower.pid_state[key] = 0.0

    motor_functions.turn_until_line_on_sensors(direction)
    line_follower.pid_state["turn_end_time"] = utime.ticks_ms()
    print(f"\n[Turn] Turn {direction} complete")

    utime.sleep_ms(20)

    ## Post-turn recovery if sensors aren't aligned
    if sensor_state[1] == 1 and sensor_state[2] == 0:
        print("[Recovery] Right sensor off line, correcting right...")
        recovery_start = utime.ticks_ms()
        while utime.ticks_diff(utime.ticks_ms(), recovery_start) < 200:
            if sensor_state[1] == 1 and sensor_state[2] == 1:
                print("[Recovery] Line recovered")
                break
            motor_functions.set_motor_speed(255, 1, 255, 0)
            utime.sleep_ms(10)
        motor_functions.stop_motors()

    elif sensor_state[1] == 0 and sensor_state[2] == 1:
        print("[Recovery] Left sensor off line, correcting left...")
        recovery_start = utime.ticks_ms()
        while utime.ticks_diff(utime.ticks_ms(), recovery_start) < 200:
            if sensor_state[1] == 1 and sensor_state[2] == 1:
                print("[Recovery] Line recovered")
                break
            motor_functions.set_motor_speed(255, 0, 255, 1)
            utime.sleep_ms(10)
        motor_functions.stop_motors()


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

def follow_line_for_duration(duration_ms):
    start_time = utime.ticks_ms()
    while utime.ticks_diff(utime.ticks_ms(), start_time) < duration_ms:
        line_follower.follow_line_pid()

def return_to_start_from_rack_a(row):
    follow_line_for_duration(200)
    follow_line_until_intersections(7 - row, sensor_index=0, debounce_ms=500)
    execute_turn("left")
    follow_line_for_duration(400)
    follow_line_until_intersections(2, sensor_index=3, debounce_ms=500)
    main()

def return_to_start_from_rack_b(row):
    follow_line_for_duration(200)
    follow_line_until_intersections(7 - row, sensor_index=3, debounce_ms=500)
    execute_turn("right")
    follow_line_for_duration(400)
    follow_line_until_intersections(2, sensor_index=3, debounce_ms=500)
    motor_functions.turn_right(120)
    execute_turn("right")
    main()


def scan_qr(initial_angle, steps, direction):
    """Sweep camera to scan for QR code"""
    turn_func = motor_functions.turn_left if direction == "left" else motor_functions.turn_right
    turn_func(initial_angle)

    for i in range(steps):
        utime.sleep_ms(400)
        turn_func(i)
        code = sensors.get_tiny_code()
        if code and code != "No QR code detected":
            return code
    return None

def get_qr_code(initial_turn_angle, scan_steps, scan_direction):
    """Scan for QR code with bidirectional sweep and fallback"""
    global boxes_collected

    sensors.enable_qr_scanning()
    utime.sleep_ms(100)

    ## First sweep
    code = scan_qr(initial_turn_angle, scan_steps, scan_direction)

    ## Try opposite direction if nothing found
    if not code or code == "No QR code detected":
        opposite = "right" if scan_direction == "left" else "left"
        if scan_direction == "left":
            motor_functions.turn_right(initial_turn_angle + scan_steps)
        else:
            motor_functions.turn_left(initial_turn_angle + scan_steps)
        utime.sleep_ms(200)
        code = scan_qr(initial_turn_angle, scan_steps, opposite)

    sensors.disable_qr_scanning()

    ## Try moving forward while scanning
    if not code or code == "No QR code detected":
        if boxes_collected >= 4:
            return None

        sensors.enable_qr_scanning()
        forward_start = utime.ticks_ms()
        while utime.ticks_diff(utime.ticks_ms(), forward_start) < 2000:
            line_follower.follow_line_pid()
            code = sensors.get_tiny_code()
            if code and code != "No QR code detected":
                break
            utime.sleep_ms(50)
        sensors.disable_qr_scanning()

        ## Use default bay if still nothing
        if not code or code == "No QR code detected":
            default_bay = 6 - boxes_collected
            code = f"Rack A, Bay {default_bay}, 1"

    return code

def parse_qr_code(code):
    """Parse QR code string into rack and row"""
    if not code or code == "No QR code detected":
        return "Rack A", 1

    try:
        code_parts = code.split(",")
        rack = code_parts[0].strip() if len(code_parts) > 0 else "Rack A"
        row = int(code_parts[2].strip()) if len(code_parts) > 2 else 1
        return rack, row
    except (ValueError, IndexError):
        return "Rack A", 1

def approach_box():
    """Navigate to box using distance sensor"""
    distance = sensors.get_vl53l0x_distance()
    while distance is not None and distance > 100:
        line_follower.follow_line_pid()
        distance = sensors.get_vl53l0x_distance()
        utime.sleep_ms(10)

def lift_box():
    """Physical box pickup sequence"""
    motor_functions.move(speed=255, direction=1, duration_ms=500)
    linear_actuator.actuator.move(linear_actuator.RETRACT_DIRECTION, 100)
    utime.sleep_ms(700)
    linear_actuator.actuator.stop()

def collect_box(initial_turn_angle=5, scan_steps=7, scan_direction="left"):
    """Complete box collection sequence"""
    global boxes_collected

    code = get_qr_code(initial_turn_angle, scan_steps, scan_direction)
    if not code:
        return None

    rack, row = parse_qr_code(code)
    approach_box()
    lift_box()
    boxes_collected += 1

    return (row, rack)

def unload_at_rack(row_number, rack):
    """Unload box at specified rack location"""
    if row_number == 2:
        linear_actuator.unload_upper(motor_functions, follow_line_for_duration, execute_turn, rack)
    else:
        linear_actuator.unload_lower(motor_functions, follow_line_for_duration, execute_turn, rack)

def navigate_to_rack(rack, row, from_point):
    """Navigate from collection point to rack"""
    motor_functions.move(speed=255, direction=0, duration_ms=300)

    if from_point in [3, 4]:  ## Left side
        motor_functions.turn_left(90)
        execute_turn("left")

        if from_point == 3:
            follow_line_for_duration(500)
            while sensor_state[0] != 1:
                line_follower.follow_line_pid()

            if rack == "Rack A":
                execute_turn("left")
                follow_line_for_duration(500)
                while sensor_state[0] != 1:
                    line_follower.follow_line_pid()
                execute_turn("right")
                follow_line_until_intersections(7 - row, sensor_index=3, debounce_ms=500)
                execute_turn("right")
            else:  ## Rack B
                execute_turn("right")
                follow_line_for_duration(500)
                follow_line_until_intersections(3, sensor_index=3, debounce_ms=200)
                execute_turn("left")
                follow_line_until_intersections(7 - row, sensor_index=0, debounce_ms=500)
                execute_turn("left")

        else:  ## Point 4
            if rack == "Rack A":
                follow_line_for_duration(2000)
                follow_line_until_intersections(7 - row, sensor_index=3, debounce_ms=500)
                execute_turn("right")
            else:  ## Rack B
                follow_line_for_duration(500)
                while sensor_state[3] != 1:
                    line_follower.follow_line_pid()
                motor_functions.move(speed=255, direction=0, duration_ms=300)
                execute_turn("right")
                follow_line_for_duration(700)
                follow_line_until_intersections(4, sensor_index=3, debounce_ms=200)
                execute_turn("left")
                follow_line_until_intersections(7 - row, sensor_index=0, debounce_ms=500)
                execute_turn("left")

    else:  ## Right side (points 5, 6)
        motor_functions.turn_right(90)
        execute_turn("right")
        follow_line_for_duration(500)
        while sensor_state[3] != 1:
            line_follower.follow_line_pid()

        if from_point == 5:
            if rack == "Rack A":
                execute_turn("left")
                follow_line_for_duration(500)
                follow_line_until_intersections(3, sensor_index=0, debounce_ms=200)
                execute_turn("right")
                follow_line_for_duration(500)
                follow_line_until_intersections(7 - row, sensor_index=0, debounce_ms=500)
                execute_turn("left")
            else:  ## Rack B
                execute_turn("right")
                follow_line_for_duration(500)
                while sensor_state[0] != 1:
                    line_follower.follow_line_pid()
                follow_line_for_duration(500)
                follow_line_until_intersections(7 - row, sensor_index=3, debounce_ms=500)
                execute_turn("left")

        else:  ## Point 6
            motor_functions.turn_left(180)
            execute_turn("left")

            if rack == "Rack B":
                follow_line_for_duration(2000)
                follow_line_until_intersections(7 - row, sensor_index=0, debounce_ms=500)
                execute_turn("left")
            else:  ## Rack A
                follow_line_for_duration(500)
                while sensor_state[0] != 1:
                    line_follower.follow_line_pid()
                execute_turn("left")
                follow_line_for_duration(500)
                follow_line_until_intersections(4, sensor_index=0, debounce_ms=200)
                execute_turn("right")
                follow_line_for_duration(500)
                follow_line_until_intersections(7 - row, sensor_index=3, debounce_ms=500)
                execute_turn("right")

    unload_at_rack(row, rack)

    if rack == "Rack A":
        return_to_start_from_rack_a(row)
    else:
        return_to_start_from_rack_b(row)

def process_collection_point(point_num, scan_direction="left"):
    """Generic collection point handler"""
    result = collect_box(initial_turn_angle=5, scan_steps=5 if point_num <= 4 else 7, scan_direction=scan_direction)

    if result:
        row, rack = result
        navigate_to_rack(rack, row, point_num)
        return True
    return False

def main():
    """Main navigation routine through all 4 collection points"""
    follow_line_until_intersections(1, sensor_index=0, debounce_ms=200)
    leds.turn_on_flashing_led()

    ## Point 3 (left side, first)
    execute_turn("left")
    if not process_collection_point(3):
        motor_functions.turn_right(30)

    ## Point 4 (left side, second)
    execute_turn("right")
    follow_line_for_duration(500)
    while sensor_state[0] != 1:
        line_follower.follow_line_pid()
    execute_turn("left")
    if not process_collection_point(4):
        motor_functions.turn_left(30)

    ## Point 5 (right side, first)
    execute_turn("left")
    follow_line_for_duration(1000)
    follow_line_until_intersections(3, sensor_index=3, debounce_ms=300)
    execute_turn("right")
    if not process_collection_point(5, scan_direction="right"):
        motor_functions.turn_left(30)

    ## Point 6 (right side, second)
    execute_turn("left")
    follow_line_for_duration(500)
    while sensor_state[3] != 1:
        line_follower.follow_line_pid()
    execute_turn("right")
    if not process_collection_point(6, scan_direction="right"):
        motor_functions.turn_right(30)

    ## Return to start
    execute_turn("right")
    follow_line_for_duration(500)
    follow_line_until_intersections(2, sensor_index=0, debounce_ms=300)
    execute_turn("left")

def initialize_robot():
    """Initialize all robot systems"""
    utime.sleep(0.4)
    leds.turn_off_flashing_led()

    ## Start sensor background thread
    try:
        _thread.start_new_thread(sensors.run_all_sensors, ())
    except OSError:
        pass

    utime.sleep(2)
    linear_actuator.calibrate_actuator()

def run_mission():
    """Execute complete box collection mission"""
    global boxes_collected
    boxes_collected = 0

    utime.sleep(0.5)
    follow_line_until_intersections(2, sensor_index=0, debounce_ms=200)
    execute_turn("left")
    main()

if __name__ == "__main__":
    initialize_robot()
    button = Pin(28, Pin.IN, Pin.PULL_DOWN)

    try:
        while True:
            if button.value() == 1:
                try:
                    run_mission()
                except Exception as e:
                    print(f"Mission error: {e}")
                    motor_functions.stop_motors()
                    leds.turn_off_flashing_led()

    except KeyboardInterrupt:
        leds.turn_off_flashing_led()
    finally:
        try:
            motor_functions.stop_motors()
        except Exception:
            pass
