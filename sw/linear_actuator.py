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

# --- Test Function ---
def test_actuator():
    while True:
        print("Extending for 2 seconds")
        actuator.move(EXTEND_DIRECTION, 80)
        sleep(2)

        print("Stopping for 2 seconds")
        actuator.stop()
        sleep(2)

        print("Retracting for 2 seconds")
        actuator.move(RETRACT_DIRECTION, 80)
        sleep(2)
        
        print("Stopping for 2 seconds")
        actuator.stop()
        sleep(2)

if __name__ == "__main__":
    test_actuator()

