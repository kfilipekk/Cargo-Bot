from time import sleep
from machine import I2C, Pin

from libs.tiny_code_reader.tiny_code_reader import TinyCodeReader

def test() -> None:
    print("Starting tiny code reader...")
    i2c_bus = I2C(id=0, scl=Pin(17), sda=Pin(16), freq=400000) ## I2C0 on GP16 & GP17

    i2c_devs = i2c_bus.scan()
    assert i2c_devs == [12, 41, 65]

    tiny_code_reader = TinyCodeReader(i2c_bus)

    print("Polling!")

    # Keep looping and reading the sensor - a real application may do this in
    # a separate thread or a few times when it expects to find a QR code
    while True:
        sleep(TinyCodeReader.TINY_CODE_READER_DELAY)

        code = tiny_code_reader.poll()
        if code is not None:
            print(f"Code found: {code}")


if __name__ == "__main__":
    test()