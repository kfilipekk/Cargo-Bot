from time import sleep, time
from machine import I2C, Pin

from libs.tiny_code_reader.tiny_code_reader import TinyCodeReader
from libs.DFRobot_TMF8x01.DFRobot_TMF8x01 import DFRobot_TMF8701
# Note: VL53L0X is not used in the main logic for now.
# from libs.VL53L0X.VL53L0X import VL53L0X

# --- I2C Bus and Sensor Initialization ---
i2c_bus = I2C(0, scl=Pin(17), sda=Pin(16), freq=100000)  # Reduced freq for stability

# --- Global sensor objects ---
qr_reader = None
tof_sensor = None

def init_sensors():
    """
    Initializes all I2C sensors and makes them ready for use.
    This should be called once at the beginning of the main script.
    """
    global qr_reader, tof_sensor
    
    print("Initializing I2C sensors...")
    
    # Initialize QR Code Reader
    try:
        qr_reader = TinyCodeReader(i2c_bus)
        print("TinyCodeReader (QR) initialized.")
    except Exception as e:
        print(f"Error initializing TinyCodeReader: {e}")

    # Initialize Time-of-Flight (ToF) Distance Sensor
    try:
        tof_sensor = DFRobot_TMF8701(i2c_bus=i2c_bus)
        if tof_sensor.begin() != 0:
            print("TMF8701 (ToF) failed to initialize.")
            tof_sensor = None
        else:
            tof_sensor.start_measurement(calib_m=tof_sensor.eMODE_NO_CALIB, mode=tof_sensor.ePROXIMITY)
            print("TMF8701 (ToF) initialized and started.")
    except Exception as e:
        print(f"Error initializing TMF8701: {e}")
        tof_sensor = None

def read_qr_code(timeout_s=5):
    """
    Polls the QR code reader for a specified duration.
    Returns the QR code string if found, otherwise None.
    """
    if not qr_reader:
        print("Error: QR reader not initialized.")
        return None

    print("Scanning for QR code...")
    start_time = time()
    while time() - start_time < timeout_s:
        code = qr_reader.poll()
        if code:
            print(f"QR Code found: {code}")
            # Expected format: "L3A" -> Side: L, Row: 3, Level: A
            try:
                side = code[0]
                row = int(code[1])
                level = code[2]
                if side in ['L', 'R'] and 1 <= row <= 6 and level in ['A', 'B']:
                    return {"side": side, "row": row, "level": level}
                else:
                    print(f"Warning: QR code '{code}' has invalid format.")
                    return None
            except (ValueError, IndexError):
                print(f"Warning: Could not parse QR code '{code}'.")
                return None
        sleep(0.2)
    
    print("No QR code found within timeout.")
    return None

def get_distance_mm():
    """
    Gets a single distance reading from the ToF sensor.
    Returns the distance in millimeters, or None if unavailable.
    """
    if not tof_sensor:
        print("Error: ToF sensor not initialized.")
        return None
        
    if tof_sensor.is_data_ready():
        distance = tof_sensor.get_distance_mm()
        # print(f"ToF Distance: {distance} mm") # Optional: uncomment for debugging
        return distance
    
    return None

# The old threaded functions are removed.
# The main program will call these functions directly.
# A simple test function can be added for diagnostics.
def test_sensors():
    init_sensors()
    print("\n--- Testing ToF Sensor ---")
    for _ in range(5):
        dist = get_distance_mm()
        if dist is not None:
            print(f"Distance: {dist} mm")
        sleep(0.5)
        
    print("\n--- Testing QR Reader ---")
    print("Please present a QR code to the sensor...")
    qr_data = read_qr_code(timeout_s=10)
    if qr_data:
        print(f"Successfully parsed QR data: {qr_data}")
    else:
        print("Failed to read or parse QR code in test.")

if __name__ == "__main__":
    test_sensors()