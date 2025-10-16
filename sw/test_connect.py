from machine import Pin, SoftI2C, I2C

#i2c = I2C(0, sda=Pin(16), scl=Pin(17))
#i2c_bus = I2C(id=0, sda=Pin(16), scl=Pin(17), freq=100000) # I2C0 on GP8 & GP9
i2c_bus = SoftI2C(sda=Pin(16), scl=Pin(17), freq=100000)
print(i2c_bus.scan()) #65 = 0x41
#print(i2c.scan())