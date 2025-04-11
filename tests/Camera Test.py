from picamera import PiCamera
import time

camera = PiCamera()

camera.resolution = (1280, 720)
camera.vflip = True

camera.start_preview()
time.sleep(2)

camera.capture('/home/speck/Speck/tests/image.jpg')

camera.stop_preview()
camera.close()
