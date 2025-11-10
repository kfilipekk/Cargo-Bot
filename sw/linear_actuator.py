from machine import Pin, PWM
from utime import sleep

MAX_DUTY_CYCLE = 65535
EXTEND_DIRECTION = 0
RETRACT_DIRECTION = 1

class Actuator:
    def __init__(self, dir_pin, pwm_pin):
        self.mDir = Pin(dir_pin, Pin.OUT)
        self.pwm = PWM(Pin(pwm_pin))
        self.pwm.freq(1000)
        self.stop()

    def move(self, direction, speed_percent):
        speed_percent = max(0, min(100, speed_percent))
        duty = int(MAX_DUTY_CYCLE * (speed_percent / 100))
        self.mDir.value(direction)
        self.pwm.duty_u16(duty)

    def stop(self):
        self.pwm.duty_u16(0)



actuator = Actuator(dir_pin=0, pwm_pin=1)

def calibrate_actuator():
    print("\n[Calibration] Starting actuator calibration...")
    ##Full extension establishes known reference position
    print("[Calibration] Extending actuator to maximum...")
    actuator.move(EXTEND_DIRECTION, 100)
    sleep(7.0)
    actuator.stop()
    print("[Calibration] Actuator fully extended")
    sleep(0.5)
    ##Partial retraction brings actuator to neutral starting position
    print("[Calibration] Retracting to neutral position...")
    actuator.move(RETRACT_DIRECTION, 100)
    sleep(1.4)
    actuator.stop()
    print("[Calibration] Calibration complete - actuator at neutral position\n")


def lift_box(level, speed=100):
    duration = 2.0 if level == "upper" else 1.0
    print(f"Lifting to {level} level...")
    actuator.move(EXTEND_DIRECTION, speed)
    sleep(duration)
    actuator.stop()

def release_box(level, speed=100):
    duration = 2.0 if level == "upper" else 1.0
    print(f"Releasing at {level} level...")
    actuator.move(RETRACT_DIRECTION, speed)
    sleep(duration)
    actuator.stop()

def lift_to_transport_height(speed=100):
    print("Lifting to transport height...")
    actuator.move(EXTEND_DIRECTION, speed)
    sleep(0.5)
    actuator.stop()

def lower_level_height():
    print("Moving to lower level height...")
    actuator.move(RETRACT_DIRECTION, 100)
    sleep(1.3)
    actuator.stop()
    print("Reached lower level height")



def upper_level_height():
    print("Moving to upper level height...")
    actuator.move(RETRACT_DIRECTION, 100)
    sleep(5.0)
    actuator.stop()
    print("Reached upper level height")

def return_from_lower_rack_to_mid():
    print("Returning from lower rack to neutral...")
    actuator.move(EXTEND_DIRECTION, 100)
    sleep(1.1)
    actuator.stop()
    print("Returned to neutral height")

def fix_linear_actuator():
    print("Manual actuator adjustment...")
    actuator.move(EXTEND_DIRECTION, 30)
    sleep(10.0)
    actuator.stop()

def line_to_rack_movement(motor_functions, follow_line_for_duration):
    follow_line_for_duration(400)
    motor_functions.stop_motors()



def unload_lower(motor_functions, follow_line_for_duration, execute_turn, rack):
    print("\n[Unload Lower] Starting lower rack unload sequence...")
    line_to_rack_movement(motor_functions, follow_line_for_duration)
    lower_level_height()
    follow_line_for_duration(450)
    motor_functions.stop_motors()
    actuator.move(EXTEND_DIRECTION, 100)
    sleep(0.5)
    print("[Unload Lower] Box lowered onto rack")
    actuator.stop()
    print("[Unload Lower] Backing away from rack")
    motor_functions.move(speed=200, direction=0, duration_ms=700)
    motor_functions.stop_motors()
    calibrate_actuator()
    if rack == "Rack A":
        print("[Unload Lower] Turning right to get back on line")
        execute_turn("right")
    elif rack == "Rack B":
        print("[Unload Lower] Turning left to get back on line")
        execute_turn("left")
    print("[Unload Lower] Following line back to intersection")
    follow_line_for_duration(400)
    print("[Unload Lower] Lower rack unload complete\n")



def unload_upper(motor_functions, follow_line_for_duration, execute_turn, rack):
    print("\n[Unload Upper] Starting upper rack unload sequence...")
    line_to_rack_movement(motor_functions, follow_line_for_duration)
    upper_level_height()
    follow_line_for_duration(450)
    motor_functions.stop_motors()
    actuator.move(EXTEND_DIRECTION, 100)
    sleep(0.5)
    print("[Unload Upper] Box lowered onto rack")
    actuator.stop()
    print("[Unload Upper] Backing away from rack")
    motor_functions.move(speed=200, direction=0, duration_ms=700)
    motor_functions.stop_motors()
    calibrate_actuator()
    if rack == "Rack A":
        print("[Unload Upper] Turning right to get back on line")
        execute_turn("right")
    elif rack == "Rack B":
        print("[Unload Upper] Turning left to get back on line")
        execute_turn("left")
    print("[Unload Upper] Following line back to intersection")
    follow_line_for_duration(400)
    print("[Unload Upper] Upper rack unload complete\n")



def test_actuator():
    while True:
        print("Extending for 2 seconds")
        actuator.move(EXTEND_DIRECTION, 80)
        sleep(2)
        print("Stopping for 2 seconds")
        actuator.stop()
        sleep(2)

if __name__ == "__main__":
    test_actuator()