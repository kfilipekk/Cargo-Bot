from time import sleep
from machine import I2C, Pin
import _thread
import utime

from libs.tiny_code_reader.tiny_code_reader import TinyCodeReader
from libs.DFRobot_TMF8x01.DFRobot_TMF8x01 import DFRobot_TMF8701
from libs.VL53L0X.VL53L0X import VL53L0X

## I2C Setup
i2c_bus = I2C(id=0, scl=Pin(17), sda=Pin(16), freq=400000)  ## I2C0 on GP16 & GP17

# Shared I2C sensor data - accessible from other modules
last_qr_code = None
last_tmf8701_distance = None
last_vl53l0x_distance = None

def get_tiny_code():
    """Get the latest QR code reading from the background sensor thread"""
    return last_qr_code

def get_tmf8701_distance():
    """Get the latest TMF8701 distance reading from the background sensor thread"""
    return last_tmf8701_distance

def get_vl53l0x_distance():
    """Get the latest VL53L0X distance reading from the background sensor thread"""
    return last_vl53l0x_distance

def run_all_sensors():
    """Run all sensors (I2C and line sensors) in a single thread"""
    global last_qr_code, last_tmf8701_distance, last_vl53l0x_distance, sensor_state

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

    print("All I2C sensors initialized - starting polling loop")

    # Main sensor polling loop
    tiny_code_counter = 0
    last_qr_code = "No QR code detected"
    line_sensor_counter = 0

    while True:
        # Poll TinyCodeReader (less frequently)
        tiny_code_counter += 1
        if tiny_code_counter >= int(TinyCodeReader.TINY_CODE_READER_DELAY / 0.1):
            code = tiny_code_reader.poll()
            if code is not None:
                last_qr_code = code
            tiny_code_counter = 0

        # Poll TMF8701
        if tof.is_data_ready():
            last_tmf8701_distance = tof.get_distance_mm()

        # Poll VL53L0X
        last_vl53l0x_distance = vl53l0.read()

        # Update line sensors frequently (every iteration)
        new_values = read_all_sensors()
        for i in range(4):
            sensor_state[i] = new_values[i]

        # Print all sensor data on one line, overwriting previous output
        qr_str = str(last_qr_code) if last_qr_code else "None"
        tmf_str = f"{last_tmf8701_distance}mm" if last_tmf8701_distance is not None else "N/A"
        vl53_str = f"{last_vl53l0x_distance}mm" if last_vl53l0x_distance is not None else "N/A"
        print(f"\rQR: {qr_str:<20} | TMF8701: {tmf_str:<8} | VL53L0X: {vl53_str:<8} | Line: {sensor_state}    ", end="")

        sleep(0.01)  # Fast polling interval for line sensors

# Keep the old function name for backwards compatibility
def run_i2c_sensors():
    """Deprecated: Use run_all_sensors() instead"""
    run_all_sensors()


## Line Sensor Setup
sensor_1 = Pin(10, Pin.IN, Pin.PULL_DOWN)  ## Leftmost
sensor_2 = Pin(8, Pin.IN, Pin.PULL_DOWN)  ## Centre-left
sensor_3 = Pin(9, Pin.IN, Pin.PULL_DOWN)  ## Centre-right
sensor_4 = Pin(11, Pin.IN, Pin.PULL_DOWN)  ## Rightmost

sensor_state = [0, 0, 0, 0]

def read_all_sensors(): return list(sensor.value() for sensor in (sensor_1, sensor_2, sensor_3, sensor_4))

def sensor_update_thread():
    """Background thread to continuously update sensor state."""
    global sensor_state
    while True:
        new_values = read_all_sensors()
        # Update in-place to maintain references in other modules
        for i in range(4):
            sensor_state[i] = new_values[i]
        # Print sensor state with carriage return to overwrite
        print(f"\rSensors: {sensor_state}", end='')
        utime.sleep(0.01)

if __name__ == "__main__":
    print("Starting sensor test...")
    # Start the I2C sensor thread
    _thread.start_new_thread(run_i2c_sensors, ())
    print("I2C sensor thread started, waiting for initialization...")
    sleep(3)  # Wait for sensors to initialize

    print("\nSensor readings:")
    while True:
        qr = get_tiny_code()
        tmf = get_tmf8701_distance()
        vl53 = get_vl53l0x_distance()
        line = read_all_sensors()
        print(f"\nQR: {qr} | TMF8701: {tmf}mm | VL53L0X: {vl53}mm | Line: {line}")
        sleep(1)