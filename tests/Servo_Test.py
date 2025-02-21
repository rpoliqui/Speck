from gpiozero import AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory
import time

factory = PiGPIOFactory()  # define pin factory to use servos for more accurate servo control

test_servo = AngularServo(14, min_pulse_width=0.0006, max_pulse_width=0.0023)
while True:
    test_servo.min()
    time.sleep(5)
    test_servo.max()
