from machine import Pin, ADC
import time

potentiometer_pin = Pin(26, Pin.IN)
adc = ADC(potentiometer_pin)

while True:
    pot_value = adc.read_u16()
    # convert to value between -0.1 - +0.1
    trim = (0.5 - pot_value / 65_536) / 5
    print(trim)
    time.sleep(0.1)