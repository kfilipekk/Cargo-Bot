from machine import Pin, PWM

from utime import sleep



# --- Constants ---

MAX_DUTY_CYCLE = 65535

EXTEND_DIRECTION = 0

RETRACT_DIRECTION = 1



# --- Actuator Class ---

class Actuator:

    def __init__(self, dir_pin, pwm_pin):

        self.mDir = Pin(dir_pin, Pin.OUT)

        self.pwm = PWM(Pin(pwm_pin))

        self.pwm.freq(1000)

        self.stop()



    def move(self, direction, speed_percent):

        """

        Moves the actuator in a given direction at a certain speed.

        :param direction: EXTEND_DIRECTION (0) or RETRACT_DIRECTION (1)

        :param speed_percent: Speed from 0 to 100

        """

        speed_percent = max(0, min(100, speed_percent)) # Clamp speed between 0-100

        duty = int(MAX_DUTY_CYCLE * (speed_percent / 100))

        self.mDir.value(direction)

        self.pwm.duty_u16(duty)



    def stop(self):

        """Stops the actuator."""

        self.pwm.duty_u16(0)



# --- Singleton Actuator Instance ---

# Initialize one actuator for the robot.

actuator = Actuator(dir_pin=0, pwm_pin=1)


# --- Calibration Function ---

def calibrate_actuator():

    """

    Calibration sequence: Fully extend the actuator, then retract for 1000ms.

    This ensures the actuator starts from a known position.

    """

    print("\n[Calibration] Starting actuator calibration...")



    # Fully extend the actuator (assuming this takes ~10 seconds to reach full extension)

    print("[Calibration] Extending actuator to maximum...")

    actuator.move(EXTEND_DIRECTION, 100)

    sleep(7.0)  # Run for 10 seconds to ensure full extension

    actuator.stop()

    print("[Calibration] Actuator fully extended")



    # Brief pause

    sleep(0.5)



    # Retract for 1000ms to set neutral position

    print("[Calibration] Retracting for 1000ms to neutral position...")

    actuator.move(RETRACT_DIRECTION, 100)

    sleep(1.4)

    actuator.stop()



    print("[Calibration] Calibration complete - actuator at neutral position\n")


# --- High-Level Functions ---

def lift_box(level, speed=100):

    """

    Lifts the cargo platform to a specified level.

    'upper' for level A, 'lower' for level B pickup.

    """

    # This requires calibration. Assuming extend = lift.

    # We need to know how long to run the motor for.

    duration = 2.0 if level == "upper" else 1.0 # Placeholder durations

    print(f"Lifting to {level} level...")

    actuator.move(EXTEND_DIRECTION, speed)

    sleep(duration)

    actuator.stop()



def release_box(level, speed=100):

    """

    Lowers the cargo platform to release a box.

    """

    # Assuming retract = lower.

    duration = 2.0 if level == "upper" else 1.0 # Placeholder durations

    print(f"Releasing at {level} level...")

    actuator.move(RETRACT_DIRECTION, speed)

    sleep(duration)

    actuator.stop()



def lift_to_transport_height(speed=100):

    """Lifts the box just enough to clear the ground for transport."""

    print("Lifting to transport height...")

    actuator.move(EXTEND_DIRECTION, speed)

    sleep(0.5) # Short duration, placeholder

    actuator.stop()



# --- Rack Unloading Functions ---

def lower_level_height():

    """Move forklift from neutral to L1 (lower rack) height."""

    print("Moving to lower level height...")

    actuator.move(RETRACT_DIRECTION, 100)  # Move to L1

    sleep(1.3)

    actuator.stop()

    print("Reached lower level height")



def upper_level_height():

    """Move forklift from L1 height to L2 (upper rack) height."""

    print("Moving to upper level height...")

    actuator.move(RETRACT_DIRECTION, 100)  # Move to L2

    sleep(5.0)

    actuator.stop()

    print("Reached upper level height")



def return_from_lower_rack_to_mid():

    """Move forklift from L1 height back to neutral."""

    print("Returning from lower rack to neutral...")

    actuator.move(EXTEND_DIRECTION, 100)

    sleep(1.1)

    actuator.stop()

    print("Returned to neutral height")



def fix_linear_actuator():

    """Manual adjustment function - moves actuator up slowly."""

    print("Manual actuator adjustment...")

    actuator.move(EXTEND_DIRECTION, 30)

    sleep(10.0)

    actuator.stop()



# --- Complete Unloading Functions ---

def line_to_rack_movement(motor_functions, follow_line_for_duration):

    """Move from intersection to rack position."""

    follow_line_for_duration(400)

    motor_functions.stop_motors()



def unload_lower(motor_functions, follow_line_for_duration, execute_turn, rack):

    """

    Complete unloading sequence for lower rack (L1).

    Assumes starting at intersection facing the correct rack.

    Returns at the same position.

    rack: "Rack A" or "Rack B" to determine turn direction

    """

    print("\n[Unload Lower] Starting lower rack unload sequence...")



    # Move to rack

    line_to_rack_movement(motor_functions, follow_line_for_duration)



    # Move to L1 height

    lower_level_height()



    # Move forward to rack

    follow_line_for_duration(450)

    motor_functions.stop_motors()



    # Lower slightly to place box

    actuator.move(EXTEND_DIRECTION, 100)

    sleep(0.5)

    print("[Unload Lower] Box lowered onto rack")

    actuator.stop()



    # Back away from rack

    print("[Unload Lower] Backing away from rack")

    motor_functions.move(speed=200, direction=0, duration_ms=700)



    # Return to neutral height

    # actuator.move(EXTEND_DIRECTION, 100)

    # sleep(1.2)

    # actuator.stop()



    motor_functions.stop_motors()

    calibrate_actuator()



    # Turn to get back on line

    if rack == "Rack A":

        print("[Unload Lower] Turning right to get back on line")

        execute_turn("right")

    elif rack == "Rack B":

        print("[Unload Lower] Turning left to get back on line")

        execute_turn("left")



    # Follow line back to intersection

    print("[Unload Lower] Following line back to intersection")

    follow_line_for_duration(400)



    print("[Unload Lower] Lower rack unload complete\n")



def unload_upper(motor_functions, follow_line_for_duration, execute_turn, rack):

    """

    Complete unloading sequence for upper rack (L2).

    Assumes starting at intersection facing the correct rack.

    Returns at the same position.

    rack: "Rack A" or "Rack B" to determine turn direction

    """

    print("\n[Unload Upper] Starting upper rack unload sequence...")



    # Move to rack

    line_to_rack_movement(motor_functions, follow_line_for_duration)



    # Move to L2 height

    upper_level_height()



    # Move forward to rack

    follow_line_for_duration(450)

    motor_functions.stop_motors()



    # Lower slightly to place box

    actuator.move(EXTEND_DIRECTION, 100)

    sleep(0.5)

    print("[Unload Upper] Box lowered onto rack")

    actuator.stop()



    # Back away from rack

    print("[Unload Upper] Backing away from rack")

    motor_functions.move(speed=200, direction=0, duration_ms=700)



    # Return to neutral height

    # actuator.move(EXTEND_DIRECTION, 100)

    # sleep(6.0)

    # actuator.stop()



    motor_functions.stop_motors()

    calibrate_actuator()



    # Turn to get back on line

    if rack == "Rack A":

        print("[Unload Upper] Turning right to get back on line")

        execute_turn("right")

    elif rack == "Rack B":

        print("[Unload Upper] Turning left to get back on line")

        execute_turn("left")



    # Follow line back to intersection

    print("[Unload Upper] Following line back to intersection")

    follow_line_for_duration(400)



    print("[Unload Upper] Upper rack unload complete\n")



# --- Test Function ---

def test_actuator():

    while True:

        print("Extending for 2 seconds")

        #actuator.move(RETRACT_DIRECTION, 30)
        actuator.move(EXTEND_DIRECTION, 80)
        sleep(2)


        print("Stopping for 2 seconds")

        actuator.stop()

        sleep(2)

if __name__ == "__main__":

    print("yes")

    test_actuator()

    lift_box("upper")

    print("hrfkjhreh")