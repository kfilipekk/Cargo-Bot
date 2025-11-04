from time import sleep
from machine import I2C, Pin
import _thread

from libs.tiny_code_reader.tiny_code_reader import TinyCodeReader
from libs.DFRobot_TMF8x01.DFRobot_TMF8x01 import DFRobot_TMF8701
from libs.VL53L0X.VL53L0X import VL53L0X

i2c_bus = I2C(id=0, scl=Pin(17), sda=Pin(16), freq=400000)  ## I2C0 on GP16 & GP17

# Shared sensor data - accessible from other modules
last_qr_code = None
last_tmf8701_distance = None
last_vl53l0x_distance = None

def run_all_sensors():
    """Run all I2C sensors in a single thread"""
    # Initialize TinyCodeReader
    tiny_code_reader = TinyCodeReader(i2c_bus)
    print("TinyCodeReader initialized")

    # Initialize TMF8701
    tof = DFRobot_TMF8701(i2c_bus=i2c_bus)
    while tof.begin() != 0:
        print("TMF8701: Initialisation failed")
        sleep(0.5)
    print("TMF8701: Initialisation done.")
    tof.start_measurement(calib_m=tof.eMODE_NO_CALIB, mode=tof.ePROXIMITY)

    # Initialize VL53L0X
    vl53l0 = VL53L0X(i2c_bus)
    vl53l0.set_Vcsel_pulse_period(vl53l0.vcsel_period_type[0], 18)
    vl53l0.set_Vcsel_pulse_period(vl53l0.vcsel_period_type[1], 14)
    vl53l0.start()
    print("VL53L0X initialized")

    print("All sensors initialized - starting polling loop")

    # Main sensor polling loop
    global last_qr_code, last_tmf8701_distance, last_vl53l0x_distance
    tiny_code_counter = 0
    last_qr_code = "No QR code detected"

    while True:
        # Poll TinyCodeReader (less frequently)
        tiny_code_counter += 1
        if tiny_code_counter >= int(TinyCodeReader.TINY_CODE_READER_DELAY / 0.1):
            code = tiny_code_reader.poll()
            if code is not None:
                last_qr_code = code
            # Overwrite the same line for QR code
            print(f"\rQR: {last_qr_code}                    ", end="")
            tiny_code_counter = 0

        # Poll TMF8701
        if tof.is_data_ready():
            last_tmf8701_distance = tof.get_distance_mm()
            print(f"\nTMF8701: Distance = {last_tmf8701_distance} mm")

        # Poll VL53L0X
        last_vl53l0x_distance = vl53l0.read()
        print(f"VL53L0X: Distance = {last_vl53l0x_distance}mm")

        sleep(0.1)  # Base polling interval## Start all sensors in a single thread
_thread.start_new_thread(run_all_sensors, ())

## Main thread can do other work or just sleep
print("Main thread running")
while True:
    sleep(2)