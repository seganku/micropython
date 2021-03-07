# micropython/sht20

```python3
from sht20 import sht20

sht = sht20(scl=19, sda=21, freq=400000, timeout=255, resolution=12)

print("Serial Number: ", sht.serial)
print("%.02f°C/%.02f%% humidity" % ( sht.temperature, sht.relative_humidity ))
```
