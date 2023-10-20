import utime
from Maix import GPIO
from fpioa_manager import fm

fm.register(9,fm.fpioa.GPIO0)
led_r=GPIO(GPIO.GPIO0,GPIO.OUT)

while True:
   led_r.value(0)
   utime.sleep_ms(100)
   led_r.value(1)
   utime.sleep_ms(100)
