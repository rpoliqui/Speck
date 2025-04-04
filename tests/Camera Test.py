from picamzero import Camera
import os

home_dir = os.environ['HOME']  #set the location of your home directory
cam = Camera()

cam.start_preview()
cam.take_photo(f"{home_dir}/Desktop/new_image.jpg")  #save the image to your desktop
cam.stop_preview()
