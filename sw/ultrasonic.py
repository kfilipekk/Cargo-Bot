from machine import Pin
import utime

pulse_out = Pin(14, Pin.OUT)
pulse_in = Pin(15,Pin.IN)

def ultrasonic_distance():

#sending the signal
    pulse_out.low() # sets pulse out to zero
    utime.sleep_us(2) # waits 2 microseconds for signal to settle
    pulse_out.high() # turns on pulse
    utime.sleep_us(10)# waits 10 microseconds (could be adjusted)
    pulse_out.low()

    while pulse_in.value() == 0:
        signal_low = utime.ticks_us() # measures how long signal is off for

    while pulse_in.value() == 1:
        signal_high = utime.ticks_us() # measures how long signal is on for

    time_passed = signal_high - signal_low # type: ignore

    distance = (time_passed * 0.0343)/2

    print("distance (cm) = ", distance)

    return distance

while True:
        ultrasonic_distance()
        ultrasonic_distance.sleep(1)

distance = ultrasonic_distance()
print("Ultrasonic distance:", distance)







