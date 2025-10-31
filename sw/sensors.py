from time import sleep
from machine import I2C, Pin
import _thread
import utime

from libs.tiny_code_reader.tiny_code_reader import TinyCodeReader
from libs.DFRobot_TMF8x01.DFRobot_TMF8x01 import DFRobot_TMF8701
from libs.VL53L0X.VL53L0X import VL53L0X

i2c_bus = I2C(id=0, scl=Pin(17), sda=Pin(16), freq=400000)  ## I2C0 on GP16 & GP17

def get_tiny_code():
    tiny_code_reader = TinyCodeReader(i2c_bus)
    sleep(TinyCodeReader.TINY_CODE_READER_DELAY)
    return str(tiny_code_reader.poll())

def get_tmf8701_distance():
    tof = DFRobot_TMF8701(i2c_bus=i2c_bus)
    if tof.begin() != 0:
        return 9999
    tof.start_measurement(calib_m=tof.eMODE_NO_CALIB, mode=tof.ePROXIMITY)
    if tof.is_data_ready():
        return tof.get_distance_mm()
    return 9999

def get_vl53l0x_distance():
    vl53l0 = VL53L0X(i2c_bus)
    vl53l0.set_Vcsel_pulse_period(vl53l0.vcsel_period_type[0], 18)
    vl53l0.set_Vcsel_pulse_period(vl53l0.vcsel_period_type[1], 14)
    vl53l0.start()
    distance = vl53l0.read()
    vl53l0.stop()
    return distance


## Sensor Setup
sensor_1 = Pin(10, Pin.IN, Pin.PULL_DOWN)  ## Leftmost
sensor_2 = Pin(9, Pin.IN, Pin.PULL_DOWN)  ## Centre-left
sensor_3 = Pin(8, Pin.IN, Pin.PULL_DOWN)  ## Centre-right
sensor_4 = Pin(11, Pin.IN, Pin.PULL_DOWN)  ## Rightmost

sensor_state = [0, 0, 0, 0]

def read_all_sensors(): return list(sensor.value() for sensor in (sensor_1, sensor_2, sensor_3, sensor_4))

def sensor_update_thread():
    """Background thread to continuously update sensor state."""
    global sensor_state
    while True:
        sensor_state = read_all_sensors()
        utime.sleep(0.01)

_thread.start_new_thread(sensor_update_thread, ())