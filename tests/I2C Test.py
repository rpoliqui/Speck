import time
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
from adafruit_extended_bus import ExtendedI2C as I2C

i2c = I2C(1)
pca = PCA9685(i2c)
pca.frequency = 60

servo1 = servo.Servo(pca.channels[0])
for i in range(180):
    servo1.angle = 180 - i
    time.sleep(0.03)