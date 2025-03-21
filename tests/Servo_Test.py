from gpiozero import AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory
import time

factory = PiGPIOFactory()  # define pin factory to use servos for more accurate servo control

test_servo = AngularServo(14, min_angle=0, max_angle=180, initial_angle=0, min_pulse_width=0.0006,
                          max_pulse_width=0.0025, pin_factory=factory)
test_servo.max()
