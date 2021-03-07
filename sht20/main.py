"""
    Demo for sht20.py module code.

    Montgomery Newcom <montgomery.newcom@gmail.com>
"""
from sht20 import sht20
from machine import Timer

sht = sht20(scl=19, sda=21, freq=400000, timeout=255, resolution=14)

def poll():
    print("%.02f°C/%.02f%% humidity" % ( sht.temperature, sht.relative_humidity ))
print("Serial Number: ", sht.serial)

t = Timer(-1)
t.init(period=500, mode=Timer.PERIODIC, callback=lambda t:poll())

