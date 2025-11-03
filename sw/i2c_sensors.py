from time import sleep
from machine import I2C, Pin
import _thread

from libs.tiny_code_reader.tiny_code_reader import TinyCodeReader
from libs.DFRobot_TMF8x01.DFRobot_TMF8x01 import DFRobot_TMF8701
from libs.VL53L0X.VL53L0X import VL53L0X

i2c_bus = I2C(id=0, scl=Pin(17), sda=Pin(16), freq=400000)  ## I2C0 on GP16 & GP17

def run_tiny_code_reader():
    tiny_code_reader = TinyCodeReader(i2c_bus)
    print("TinyCodeReader thread started")
    while True:
        sleep(TinyCodeReader.TINY_CODE_READER_DELAY)
        code = tiny_code_reader.poll()
        if code is not None:
            print(f"TinyCodeReader: Code found: {code}")

def run_tmf8701():
    tof = DFRobot_TMF8701(i2c_bus=i2c_bus)
    while tof.begin() != 0:
        print("TMF8701: Initialisation failed")
        sleep(0.5)
    print("TMF8701: Initialisation done.")
    tof.start_measurement(calib_m=tof.eMODE_NO_CALIB, mode=tof.ePROXIMITY)
    while True:
        if tof.is_data_ready():
            print(f"TMF8701: Distance = {tof.get_distance_mm()} mm")
        sleep(0.5)

def run_vl53l0x():
    vl53l0 = VL53L0X(i2c_bus)
    vl53l0.set_Vcsel_pulse_period(vl53l0.vcsel_period_type[0], 18)
    vl53l0.set_Vcsel_pulse_period(vl53l0.vcsel_period_type[1], 14)
    while True:
        vl53l0.start()
        for _ in range(10):
            distance = vl53l0.read()
            print(f"VL53L0X: Distance = {distance}mm")
            sleep(0.5)
        vl53l0.stop()

## Start each sensor in its own thread
_thread.start_new_thread(run_tiny_code_reader, ())
_thread.start_new_thread(run_tmf8701, ())
_thread.start_new_thread(run_vl53l0x, ())

## Main thread can do other work or just sleep
while True:
    sleep(2)