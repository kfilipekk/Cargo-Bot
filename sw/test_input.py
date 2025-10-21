from machine import Pin
import utime

def test_input_poll(pin):
    "Simple poll of input"
    input_pin = pin
    input = Pin(input_pin, Pin.IN, Pin.PULL_DOWN) ##Pull down for 0 initially
    start = utime.ticks_ms()

    while utime.ticks_diff(utime.ticks_ms(), start) < 500:
        print(utime.ticks_diff(utime.ticks_ms(), start))
        value = input.value() ##Polling
        print(f"Input = {value}")
        utime.sleep(0.2)
    print(f"Finished testing pin {pin}")


def input_irq(p):
    "Interrupt handler"
    # print(p)
    value = p.value()
    print(f"Input changed, value={value}")


def test_input_irq():
    "More advanced, interrupt based input handling"
    input_pin = 18  # Pin 18 = GP18 (labelled 24 on the jumper)
    input = Pin(input_pin, Pin.IN, Pin.PULL_DOWN) # Think carefully whether you need pull up or pull down
    input.irq(handler=input_irq) # Register irq, you could also consider rising and falling edges c.f. https://docs.micropython.org/en/latest/library/machine.Pin.html

    while True:
        pass # irq handling does the rest in this instance


if __name__ == "__main__":
    test_input_poll()
    # test_input_irq()