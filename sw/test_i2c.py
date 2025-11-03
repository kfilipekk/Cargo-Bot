from time import sleep
from machine import I2C, Pin

from libs.tiny_code_reader.tiny_code_reader import TinyCodeReader
from libs.DFRobot_TMF8x01.DFRobot_TMF8x01 import DFRobot_TMF8701
from libs.VL53L0X.VL53L0X import VL53L0X

def test():
    print("Starting i2c sensors")
    i2c_bus = I2C(id=0, scl=Pin(17), sda=Pin(16), freq=400000) # I2C0 on GP16 & GP17

    i2c_devs = i2c_bus.scan()
    print(i2c_devs)
    assert len(i2c_devs) == 3
    assert i2c_devs == [12, 41, 65]

if __name__ == "__main__":
    test()