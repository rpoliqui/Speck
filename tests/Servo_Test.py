from gpiozero import AngularServo
import time

test_servo = AngularServo(14, min_pulse_width=0.0006, max_pulse_width=0.00025)
while True:
    test_servo.min()
    time.sleep(5)
    test_servo.max()
