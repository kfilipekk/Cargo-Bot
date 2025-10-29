"""Unified sensor interface for robot navigation and cargo detection."""
from machine import I2C, Pin
from utime import sleep, sleep_us, ticks_us
from libs.tiny_code_reader.tiny_code_reader import TinyCodeReader
from libs.DFRobot_TMF8x01.DFRobot_TMF8x01 import DFRobot_TMF8701
from libs.VL53L0X.VL53L0X import VL53L0X

## Ultrasonic sensors (side-mounted for box detection)
ultrasonic_trigger = Pin(15, Pin.OUT)
ultrasonic_echo = Pin(14, Pin.IN)

## I2C bus shared by all sensors
i2c = I2C(0, scl=Pin(17), sda=Pin(16), freq=400000)

## QR code reader
qr_reader = TinyCodeReader(i2c)

## Distance sensors
tmf8701 = DFRobot_TMF8701(i2c_bus=i2c)  ## Verify box pickup (front-mounted)
vl53l0x = VL53L0X(i2c)  ## Short range high precision (for positioning at bays)

## Initialize TMF8701
while tmf8701.begin() != 0:
    sleep(0.1)
tmf8701.start_measurement(calib_m=tmf8701.eMODE_NO_CALIB, mode=tmf8701.ePROXIMITY)

## Initialize VL53L0X
vl53l0x.set_Vcsel_pulse_period(vl53l0x.vcsel_period_type[0], 18)
vl53l0x.set_Vcsel_pulse_period(vl53l0x.vcsel_period_type[1], 14)

def check_box_present(threshold_cm=20):
    """Check if a box is present using side-mounted ultrasonic sensors."""
    ultrasonic_trigger.low()
    sleep_us(2)
    ultrasonic_trigger.high()
    sleep_us(10)
    ultrasonic_trigger.low()

    signal_low = signal_high = 0
    while ultrasonic_echo.value() == 0:
        signal_low = ticks_us()
    while ultrasonic_echo.value() == 1:
        signal_high = ticks_us()

    distance_cm = ((signal_high - signal_low) * 0.0343) / 2
    return distance_cm < threshold_cm

def verify_box_picked_up(threshold_mm=100):
    """Verify box has been picked up using front-mounted TMF8701 sensor."""
    if tmf8701.is_data_ready():
        distance = tmf8701.get_distance_mm()
        return distance < threshold_mm and distance > 0
    return False

def get_bay_distance():
    """Get precise distance to cargo bay wall using VL53L0X sensor."""
    vl53l0x.start()
    distance = vl53l0x.read()
    vl53l0x.stop()
    return distance

def scan_qr_code(timeout_attempts=10):
    """Scan QR code and return the decoded value."""
    for _ in range(timeout_attempts):
        code = qr_reader.poll()
        if code:
            return code.strip()
        sleep(TinyCodeReader.TINY_CODE_READER_DELAY)
    return None