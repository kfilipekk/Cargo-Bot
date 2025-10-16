from machine import Pin
from utime import sleep

pin = Pin("LED", Pin.OUT)
i=0
print("LED starts flashing...")
while i<5:
    try:
        i+=1
        pin.toggle()
        sleep(1)
    except KeyboardInterrupt:
        break
pin.off()
print("Finished.")
