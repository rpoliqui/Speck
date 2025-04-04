from picamzero import Camera
import os

cam = Camera()

cam.start_preview()
cam.take_photo("/home/speck/Speck/tests/new_image.jpg")  # save the image to your desktop
cam.stop_preview()
