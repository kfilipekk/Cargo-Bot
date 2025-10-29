from machine import Pin
import utime

pulse_out = Pin(15, Pin.OUT)
pulse_in = Pin(14, Pin.IN)

def distance():
    """Measure distance using ultrasonic sensor. Returns distance in cm or None on error."""
    try:
        # Send the signal
        pulse_out.low()
        utime.sleep_us(2)
        pulse_out.high()
        utime.sleep_us(10)
        pulse_out.low()

        # Wait for echo start (with timeout)
        timeout_start = utime.ticks_us()
        signal_low = 0
        while pulse_in.value() == 0:
            signal_low = utime.ticks_us()
            if utime.ticks_diff(signal_low, timeout_start) > 30000:  # 30ms timeout
                return None

        # Wait for echo end (with timeout)
        timeout_start = utime.ticks_us()
        signal_high = 0
        while pulse_in.value() == 1:
            signal_high = utime.ticks_us()
            if utime.ticks_diff(signal_high, timeout_start) > 30000:  # 30ms timeout
                return None

        time_passed = utime.ticks_diff(signal_high, signal_low)
        ultra_distance = (time_passed * 0.0343) / 2
        return ultra_distance
    except:
        return None







