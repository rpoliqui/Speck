from picamera2 import Picamera2
import time

camera = Picamera2()
camera.start()

camera.capture_file('test.jpg')

camera.close()
