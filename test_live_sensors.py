"""Live sensor data monitoring script."""

from sw import line_follower
from utime import sleep

print("=" * 60)
print("LIVE SENSOR DATA MONITOR")
print("=" * 60)
print("Hardware layout: Sensor 11, 8, 9, 10 (left to right)")
print("List format: [s1(pin11), s2(pin8), s3(pin9), s4(pin10)]")
print("             [leftmost,  center-L, center-R, rightmost]")
print("=" * 60)
print("\nPress Ctrl+C to stop\n")

try:
    while True:
        # Get sensor readings
        sensor_state = line_follower.get_sensor_state_list()

        # Create visual representation
        visual = ""
        for i, state in enumerate(sensor_state):
            if state == 1:
                visual += "[■]"  # On line (black)
            else:
                visual += "[□]"  # Off line (white)
            if i < len(sensor_state) - 1:
                visual += " "

        # Print on same line (overwrite)
        print(f"\rSensors: {sensor_state}  {visual}  ", end="")

        sleep(0.1)  # Update 10 times per second

except KeyboardInterrupt:
    print("\n\nMonitoring stopped.")
    print("=" * 60)
