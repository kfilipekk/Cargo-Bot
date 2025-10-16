from machine import Pin
from utime import sleep

led_pin = 28
led = Pin(led_pin, Pin.OUT)

while True:
  led.toggle()
  sleep(0.5)