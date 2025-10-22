print("Hello from main.py!")

# from sw import test_input
# from sw import test_tiny_code_reader
# from sw import test_TMF8x01_get_distance
# from sw import test_vl53l0x
from sw import motor
from utime import sleep

# test_tiny_code_reader.test()
# test_TMF8x01_get_distance.test()
# test_vl53l0x.test()

def test():
    print("Moving forward for 2 seconds")
    motor.MoveBackward(50)
    sleep(2)
    motor.StopMotor()
    print("Stopped")


if __name__ == "__main__":
    test()




