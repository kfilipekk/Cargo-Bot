from machine import Pin, PWM
from utime import sleep

# This setup assumes a motor driver where PWM is applied directly to the input pins.
# Motor A is on pins 5 and 4. Motor B is on pins 6 and 7.

# --- Pin Definitions ---
led = Pin(25, Pin.OUT)
# Motor A pins
motor_a_in1 = Pin(5, Pin.OUT)
motor_a_in2 = Pin(4, Pin.OUT)
# Motor B pins
motor_b_in1 = Pin(6, Pin.OUT)
motor_b_in2 = Pin(7, Pin.OUT)

# Set initial state to off
motor_a_in1.value(0)
motor_a_in2.value(0)
motor_b_in1.value(0)
motor_b_in2.value(0)

led.toggle()

# --- Motor Control Functions ---

def MoveForward(duty):
    """
    Moves both motors forward.
    Applies PWM to the 'forward' pin and sets the 'backward' pin to 0.
    """
    duty_16 = int((duty * 65536) / 100)

    # Motor A forward
    PWM(motor_a_in1, freq=1000, duty_u16=duty_16)
    motor_a_in2.value(0)

    # Motor B forward
    PWM(motor_b_in1, freq=1000, duty_u16=duty_16)
    motor_b_in2.value(0)

def MoveBackward(duty):
    """
    Moves both motors backward.
    Applies PWM to the 'backward' pin and sets the 'forward' pin to 0.
    """
    duty_16 = int((duty * 65536) / 100)

    # Motor A backward
    motor_a_in1.value(0)
    PWM(motor_a_in2, freq=1000, duty_u16=duty_16)

    # Motor B backward
    motor_b_in1.value(0)
    PWM(motor_b_in2, freq=1000, duty_u16=duty_16)

def TurnRight(duty):
    """
    Turns right: Motor A forward, Motor B backward.
    """
    duty_16 = int((duty * 65536) / 100)

    # Motor A forward
    PWM(motor_a_in1, freq=1000, duty_u16=duty_16)
    motor_a_in2.value(0)

    # Motor B backward
    motor_b_in1.value(0)
    PWM(motor_b_in2, freq=1000, duty_u16=duty_16)

def TurnLeft(duty):
    """
    Turns left: Motor A backward, Motor B forward.
    """
    duty_16 = int((duty * 65536) / 100)

    # Motor A backward
    motor_a_in1.value(0)
    PWM(motor_a_in2, freq=1000, duty_u16=duty_16)

    # Motor B forward
    PWM(motor_b_in1, freq=1000, duty_u16=duty_16)
    motor_b_in2.value(0)

def StopMotor():
    """
    Stops both motors by setting all input pins to 0.
    This re-initializes them as simple output pins.
    """
    global motor_a_in1, motor_a_in2, motor_b_in1, motor_b_in2
    motor_a_in1 = Pin(5, Pin.OUT, value=0)
    motor_a_in2 = Pin(4, Pin.OUT, value=0)
    motor_b_in1 = Pin(6, Pin.OUT, value=0)
    motor_b_in2 = Pin(7, Pin.OUT, value=0)


# --- Main Loop for Testing ---
if __name__ == "__main__":
    while True:
        try:
            duty_cycle_str = input("Enter PWM duty cycle (0-100): ")
            duty_cycle = float(duty_cycle_str)
            if not 0 <= duty_cycle <= 100:
                print("Please enter a value between 0 and 100.")
                continue

            print(f"Moving forward with {duty_cycle}% duty cycle...")
            MoveForward(duty_cycle)
            sleep(3)

            print(f"Moving backward with {duty_cycle}% duty cycle...")
            MoveBackward(duty_cycle)
            sleep(3)

            print("Stopping motors.")
            StopMotor()
            sleep(2)

        except ValueError:
            print("Invalid input. Please enter a number.")
        except KeyboardInterrupt:
            print("Exiting program.")
            StopMotor()
            break

