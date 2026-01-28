from time import sleep
import dht
from machine import Pin

sensor = dht.DHT11(Pin(5))

while True:
    try:
        sensor.measure()
        print("Temperature:", sensor.temperature())
        print("Humidity:", sensor.humidity())
    except OSError as e:
        print("Read error:", e)

    sleep(2)   # THIS LINE SAVES YOUR SANITY
