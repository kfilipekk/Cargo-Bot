from time import sleep
from machine import I2C, Pin
import _thread
import utime
red_led = Pin(26, Pin.OUT)
yellow_led = Pin(22, Pin.OUT)

def turn_on_red_led():
    red_led.value(1)

def turn_off_red_led():
    red_led.value(0)

def turn_on_flashing_led():
    yellow_led.value(0)

def turn_off_flashing_led():
    yellow_led.value(1)


