print("Hello from main.py!")

from sw import motor_functions
from sw import line_follower
from sw import sensors
from sw.sensors import sensor_state
import utime

## Turning states for intersection detection
turning_states = [
    [1, 1, 1, 1],  ## All sensors on line (4-way intersection)
    [1, 0, 0, 1],  ## Outer sensors on line
    [1, 1, 1, 0],  ## Left three sensors on line (left turn available)
    [0, 1, 1, 1],  ## Right three sensors on line (right turn available)
    [1, 0, 0, 0],  ## Only leftmost sensor on line
    [0, 0, 0, 1],  ## Only rightmost sensor on line
]



def execute_turn(direction):
    for key in line_follower.pid_state:
        line_follower.pid_state[key] = 0.0
    motor_functions.turn_until_line_on_sensors(direction)


def collect_box():
    distance_to_box = sensors.get_tmf8701_distance()
    code = sensors.get_tiny_code().split(",")
    row = code[2]

    while distance_to_box > 0:
        line_follower.run_line_follower(mode='pid', debug=False)
    ## Lift up mechanism?
    execute_turn("left")
    execute_turn("left")
    while sensor_state[0] != 1 and sensor_state[3] != 1:
        line_follower.run_line_follower(mode='pid', debug=False)
    motor_functions.stop_motors
    return row


def main():
    utime.sleep(0.5)
    motor_functions.move(speed=255, direction=1, duration_ms=100)




    ## --- Point 1 ---
    while max(sensor_state[0], sensor_state[3]) != 1:
        line_follower.run_line_follower(mode='pid', debug=False)

    ## --- Point 2 ---
    motor_functions.move(speed=255, direction=1, duration_ms=100)
    execute_turn("left")
    while sensor_state[0] != 1:
        line_follower.run_line_follower(mode='pid', debug=False)

    ## --- Point 3 ---
    execute_turn("left")
    distance_to_box = sensors.get_tmf8701_distance()
    if distance_to_box < 200:
        ## If a box is present
        row = collect_box()
        execute_turn("left")
        while sensor_state[0] != 1:
            line_follower.run_line_follower(mode='pid', debug=False)
        execute_turn("right")

        count = 0
        start_time = utime.ticks_ms()
        while count != row:
            line_follower.run_line_follower(mode='pid', debug=False)
            if sensor_state[3] == 1 and utime.ticks_ms() - start_time > 100:
                count += 1
                start_time = utime.ticks_ms()
        execute_turn("right")
        motor_functions.move(speed=255, direction=1, duration_ms=500)



    execute_turn("right")
    while sensor_state[0] != 1:
        line_follower.run_line_follower(mode='pid', debug=False)

    ## --- Point 4 ---
    execute_turn("left")
    distance_to_box = sensors.get_tmf8701_distance()
    if distance_to_box < 200:
        collect_box()





    ## --- If no boxes on the left they must be on the right ---
    execute_turn("left")
    count = 0
    start_time = utime.ticks_ms()
    while count != 3:
        line_follower.run_line_follower(mode='pid', debug=False)
        if sensor_state[3] == 1 and utime.ticks_ms() - start_time > 100:
            count += 1
            start_time = utime.ticks_ms()

    ## --- Point 5 ---
    execute_turn("right")
    distance_to_box = sensors.get_tmf8701_distance()
    if distance_to_box < 200:
        collect_box()

    execute_turn("left")
    while sensor_state[3] != 1:
        line_follower.run_line_follower(mode='pid', debug=False)

    ## --- Point 6 ---
    execute_turn("right")
    distance_to_box = sensors.get_tmf8701_distance()
    if distance_to_box < 200:
        collect_box()

    execute_turn("right")

    # --- No boxes found, go back to the start ---
    while count != 2:
        line_follower.run_line_follower(mode='pid', debug=False)
        if sensor_state[0] == 1 and utime.ticks_ms() - start_time > 100:
            count += 1
            start_time = utime.ticks_ms()
    execute_turn("left")

if __name__ == "__main__":
    while True:
        line_follower.run_line_follower(mode='pid', debug=False)
        print(sensor_state)









