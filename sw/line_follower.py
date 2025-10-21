from machine import Pin
from utime import sleep
from sw import motor

LINE_SENSOR_PINS = [2,3,4,5]
TURN_DUTY_CYCLE = 70
TURN_DURATION_S = 1

line_sensors = [Pin(pin, Pin.IN) for pin in LINE_SENSOR_PINS]

def turn_90_degrees_right() -> None:
    """Turns the robot 90 degrees to the right by spinning the wheels in opposite directions."""
    motor.TurnRight(TURN_DUTY_CYCLE)
    sleep(TURN_DURATION_S)
    motor.StopMotor()
    print("Turn complete")
    sleep(1)

def turn_90_degrees_left() -> None:
    """Turns the robot 90 degrees to the right by spinning the wheels in opposite directions."""
    motor.TurnLeft(TURN_DUTY_CYCLE)
    sleep(TURN_DURATION_S)
    motor.StopMotor()
    print("Turn complete")
    sleep(1)

def main():
    """Main loop to detect a line and react."""
    print("Starting line detection...")
    while True:
        sensor_values = [sensor.value() for sensor in line_sensors]
        print(f"Sensor values: {sensor_values}")
        if 1 in sensor_values:
            turn_90_degrees_right()
        else:
            motor.MoveForward(TURN_DUTY_CYCLE)
        sleep(0.1)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Program stopped.")
        motor.StopMotor()
