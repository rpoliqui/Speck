"""
Ryan Poliquin, started on 1/27/2025
This code contains all definitions required to control Speck. It is broken into the basic objects that
need to be controlled. These classes are combined into the top level class, Speck. All pins are defined at the beginning
of the code so that they may be modified for different implementations.
_______________________________________________________________________________________________________________________
Classes:
    Speck: The object representing Speck as a whole. All commands should be sent to this object and handled by the
           corresponding objects
    Joint: A single servo joint. Defined by the minimum and maximum the angles can move to and the starting angle of the
    joint
    Leg: A combination of three joints that form a leg
    Camera: An object that controls a Raspberry Pi Camera module and performs all operations to detect the crate
    CrateJaws: An object used to control the crate holding jaws that are used to pick up a specially designed crate
Global Variables:
    AvailablePins: a boolean array used to keep track of what pins are available. True = available, False = unavailable
    WALK_GAIT: an array of leg movements that allow Speck to walk
_______________________________________________________________________________________________________________________
References:
    https://docs.python.org/3/
    https://www.geeksforgeeks.org/python-docstrings/
    https://forums.raspberrypi.com/viewtopic.php?t=173157
    https://stackoverflow.com/questions/8247605/configuring-so-that-pip-install-can-work-from-github
    https://gpiozero.readthedocs.io/en/latest/
    https://gitpython.readthedocs.io/en/stable/tutorial.html
    https://packaging.python.org/en/latest/tutorials/packaging-projects/
    https://projects.raspberrypi.org/en/projects/getting-started-with-git/0
    https://stackoverflow.com/questions/66054625/pyinstaller-error-running-script-with-pyzmq-dependency
    https://git-scm.com/docs/git-pull
    https://docs.python.org/3/library/math.html
    https://github.com/lifeparticle/Markdown-Cheatsheet
    https://docs.python.org/3/library/threading.html#timer-objects
    https://docs.python.org/3/library/threading.html#thread-objects
    https://picamera.readthedocs.io/en/release-1.13/index.html
    https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html
    https://www.youtube.com/watch?v=40tZQPd3z8g
    https://www.hackster.io/rbnsmathew/simple-quadruped-robot-ebe1fd
    https://www.geeksforgeeks.org/print-objects-of-a-class-in-python/
    https://docs.github.com/en/repositories/releasing-projects-on-github/viewing-your-repositorys-releases-and-tags
    https://git-scm.com/docs/git-describe
    https://realpython.com/intro-to-python-threading/
    https://www.geeksforgeeks.org/queue-in-python/
    https://robotics.stackexchange.com/questions/16252/servo-motor-power-consumption-issue
    https://learnopencv.com/contour-detection-using-opencv-python-c/
    https://www.geeksforgeeks.org/python-opencv-cheat-sheet/
    https://www.tutorialspoint.com/how-to-change-the-contrast-and-brightness-of-an-image-using-opencv-in-python
    https://stackoverflow.com/questions/58632469/how-to-find-the-orientation-of-an-object-shape-python-opencv
    https://forum.opencv.org/t/remove-unwanted-contours-with-irregular-shapes/6854
    https://handmap.github.io/measuring-size-and-distance-opencv/
    https://stackoverflow.com/questions/40460873/how-to-draw-a-rectangle-by-specifying-its-4-corners
    https://gist.github.com/jdhao/1cb4c8f6561fbdb87859ac28a84b0201
    https://docs.opencv.org/4.x/dd/d49/tutorial_py_contour_features.html
    https://community.appinventor.mit.edu/t/raspberry-pi-bluetooth-send-receive/59846/3
    https://www.w3schools.com/python/python_datetime.asp
    https://support.avh.corellium.com/devices/rpi4/rpi4-ble
    https://iot.appinventor.mit.edu/iot/reference/bluetoothle
    https://www.youtube.com/watch?v=gXXRpjzrBsA
    https://www.youtube.com/watch?v=RvbWl8rZOoQ
    https://github.com/ukBaz/python-bluezero/blob/main/examples/peripheral_read_with_without_options.py
    https://www.kevsrobots.com/blog/servo-easing-with-pancake-bot.html
    https://www.youtube.com/watch?v=hGHnLUXNN9k&ab_channel=EngineerM
    https://blog.garybricks.com/control-16-servos-with-raspberry-pi-pca9685-driver
"""
# __________Import Statements__________
from __future__ import division
import numpy as np
import math
import time
import bluetooth
import cv2
import datetime
import os
from easing import Ease_out_quart
import subprocess
from picamera2 import Picamera2
from math import atan2, sin, asin, acos, sqrt, cos
from threading import Timer, Thread, Barrier, Lock
from queue import Queue
import pigpio
from gpiozero import Button, Device, OutputDevice, LED
from gpiozero.pins.pigpio import PiGPIOFactory
import Adafruit_PCA9685


# __________Pin Definition__________
# Joint Pins
PIN_RF_HIP_LAT = 0
PIN_RF_HIP_LONG = 1
PIN_RF_KNEE = 2
PIN_LF_HIP_LAT = 4
PIN_LF_HIP_LONG = 5
PIN_LF_KNEE = 6
PIN_RB_HIP_LAT = 8
PIN_RB_HIP_LONG = 9
PIN_RB_KNEE = 10
PIN_LB_HIP_LAT = 12
PIN_LB_HIP_LONG = 13
PIN_LB_KNEE = 14

# Motor Driver Pins
PIN_IN1 = 8
PIN_IN2 = 7
PIN_IN3 = 5
PIN_IN4 = 12

# Object Sensor Pins
PIN_FAR_LEFT_SENSOR = 21
PIN_LEFT_SENSOR = 26
PIN_CENTER_SENSOR = 20
PIN_RIGHT_SENSOR = 19
PIN_FAR_RIGHT_SENSOR = 16

# Limit Switch
PIN_LEFT_SWITCH = 13
PIN_RIGHT_SWITCH = 6

# LED Flash
PIN_FLASH = 4

# __________System Constants__________
HIP_LENGTH = 74  # mm
UPPER_LEG_LENGTH = 124.5  # mm
LOWER_LEG_LENGTH = 110  # mm
JAW_OPEN_TIME = 4.5  # s
JAW_CLOSE_TIME = 4.5  # s
STEP_TIME = .05  # s
SPECK_LENGTH = 181.3  # distance from center of longitudinal hip joints
SPECK_WIDTH = 276.7  # distance from outside both legs
CRATE_WIDTH = 75  # mm
JOINT_X_OFFSET = 90.65  # mm
JOINT_Y_OFFSET = 57.75  # mm
JOINT_Z_OFFSET = 47.5  # mm

# __________Global Variables__________
# Create an array of boolean values to keep track of what GPIO pins are available on the pi
# 1 = available; 0 = unavailable
AvailablePins = np.ones(40)

# __________Gait Arrays__________
# array storing changes in x, y and z positions for each leg to enable Speck to walk. Layout:
# {Step n: {[Legs], dx, dy, dz},
# {Step n+1: {[Legs], dx, dy, dz}}
# LEGS: [RF, LF, RB, LB] 4 = ALL
# DIRECTIONS: [X, Y, Z] +X = backwards, +Y = downward

STEP_SIZE = 75
STEP_RAISE = 40
VERTICAL_SHIFT = 10

WALK_GAIT = (
    ([4], 27.5, 0, 0),  # shift backwards to a stable position

    # move back left leg forward
    ([3], 0, -STEP_RAISE, 0),
    ([0, 1, 2], 0, 0, 0),
    ([3], -STEP_SIZE, 0, 0),
    ([0, 1, 2], 0, 0, 0),
    ([3], 0, STEP_RAISE + 5, 0),
    ([0, 1, 2], 0, 0, 0),

    # shift into new stable position
    ([4], -5, 0, 0),

    # move front left leg forward
    ([1], 0, -STEP_RAISE, 0),
    ([0, 3, 2], 0, 0, 0),
    ([1], -STEP_SIZE, 0, 0),
    ([0, 3, 2], 0, 0, 0),
    ([1], 0, STEP_RAISE + 5, 0),
    ([0, 3, 2], 0, 0, 0),

    # shift into new stable position
    ([4], 35, 0, 0),

    # move back left leg forward
    ([2], 0, -STEP_RAISE, 0),
    ([0, 1, 3], 0, 0, 0),
    ([2], -STEP_SIZE, 0, 0),
    ([0, 1, 3], 0, 0, 0),
    ([2], 0, STEP_RAISE + 5, 0),
    ([0, 1, 3], 0, 0, 0),

    # shift into new stable position
    ([4], -22.5, 0, 0),

    # move front left leg forward.
    ([0], 0, -STEP_RAISE, 0),
    ([2, 1, 3], 0, 0, 0),
    ([0], -STEP_SIZE, 0, 0),
    ([2, 1, 3], 0, 0, 0),
    ([0], 0, STEP_RAISE + 5, 0),
    ([2, 1, 3], 0, 0, 0),

    # shift back to starting position
    ([4], 40, 0, 0),
)

TROT = (([0, 3], -30, -30, 0),
        ([1, 2], 15, 0, 0),
        ([0, 3], -30, 30, 0),
        ([1, 2], 15, 0, 0),

        ([1, 2], -30, -30, 0),
        ([0, 3], 15, 0, 0),
        ([1, 2], -30, 30, 0),
        ([0, 3], 15, 0, 0))

# __________Environment Setup__________
factory = PiGPIOFactory()  # define pin factory to use servos for more accurate servo control
Device.pin_factory = factory

# __________PWM Controller Setup__________
pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(60)


# __________Class Definitions__________
class Joint:
    """
    The Joint class is used to represent a single joint in a leg assembly.

    :param self.pin:type int: the GPIO pin that the joint servo is connected to
    :param self.min_angle: int: the minimum angle that the joint can be set to
    :param self.max_angle: int: the maximum angle that the joint can be set to
    :param self.current_angle: int: the angle that the joint is currently at
    """

    def __init__(self, channel: int, min_angle=0.0, max_angle=180.0, starting_angle=0.0, flipped=False):
        """
        Constructor for the Joint class.
        
        :argument channel:type int: the PWM generator channel where the servo is attached
        :argument min_angle:type float: the minimum angle that the joint can be set to
        :argument max_angle:type float: the maximum angle that the joint can be set to
        :argument starting_angle:type float: the angle to set the joint to on startup
        """
        self.flipped = flipped  # keep track of if all angles need to be flipped
        self.min_angle = min_angle  # define min angle
        self.max_angle = max_angle  # define max angle
        self.current_angle = starting_angle  # set the starting angle
        self.channel = channel  # define channel where this joint is connected

        # tunable pulsewidth for accurate angle control
        self.min_pulse_width = 600
        self.max_pulse_width = 3200
        self.set_angle(starting_angle)  # properly set the starting angle of the joint

    def set_angle(self, angle: float):
        """
        A function used to set the angle of the joint.

        :argument angle:type float: the angle to set the joint to
        :return: None
        """
        # constrain angle to defined range
        angle = max(self.min_angle, min(self.max_angle, angle))

        # update angle if flipped
        if self.flipped:
            # e.g. if min=0, max=180, angle=30  -> flipped_angle = 150
            angle = self.max_angle - (angle - self.min_angle)

        # update current angle parameter
        self.current_angle = angle

        # calculate necessary pulsewidth for angle in microseconds
        span = self.max_angle - self.min_angle
        pulse_width_us = self.min_pulse_width + ((angle - self.min_angle) / span) * (
                    self.max_pulse_width - self.min_pulse_width)

        # convert pulsewidth from microseconds to PWM ticks for PCA9685
        pulse_length_us = 1000000 / 60  # 60 Hz frequency => 16,666.67 µs per cycle
        tick_length_us = pulse_length_us / 4096  # Each tick ≈ 4.069 µs
        pulse_width_ticks = int(pulse_width_us / tick_length_us)

        # set servo angle
        pwm.set_pwm(self.channel, 0, pulse_width_ticks)
        return None

    def change_angle(self, change_in_angle: float):
        """
        A function used to change the angle of the joint.

        :argument change_in_angle:type float: the amount to change the angle of the joint
        :return: None
        """
        # set the angle of the joint to the current angle plus the change
        self.set_angle(self.current_angle + change_in_angle)
        return None


class Leg:
    """
    The Leg class is used to represent a single leg of Speck. A leg is made of three Joints to control the tilt of the
    leg in all three directions to create a walking motion.

    :parameter self.hip_lat:type Joint: a Joint object representing the tilt of the leg from the body, known as the
    lateral hip joint
    :parameter self.hip_long:type Joint: a Joint object representing the main hip joint, known as the longitudinal hip
    joint
    :parameter self.knee:type Joint: a Joint object representing the knee joint
    :parameter self.current_position:type {int, int, int}: an array with the current position of the foot in the form
    {x, y, z} in millimeters.
    :parameter self.flipped:type bool: Flag to flip and angles of the leg joints
    """

    def __init__(self, hip_lat_channel: int, hip_long_channel: int, knee_channel: int, flipped=False, hip_flip=False):
        """
        Constructor for the Leg class.

        :argument hip_lat_channel:type int: The pin that the lateral hip joint servo is connected to
        :argument hip_long_channel:type int: The pin that the longitudinal hip joint servo is connected to
        :argument knee_channel:type int: The pin that the knee joint is connected to
        """
        # Create joint objects for the three joints in the leg
        # for correct functioning, the lateral hip joints needs to be flipped by default
        self.hip_lat = Joint(hip_lat_channel, min_angle=-90, max_angle=90, starting_angle=0, flipped=(flipped ^ hip_flip))
        self.hip_long = Joint(hip_long_channel, min_angle=-90, max_angle=90, starting_angle=-90, flipped=flipped)
        self.knee = Joint(knee_channel, min_angle=0, max_angle=180, starting_angle=180, flipped=flipped)

        self.flipped = flipped  # specify whether all angles need to be flipped or not
        self.current_position = [0, 0, 0]  # assume robot starts at origin until the position of the joints is set.
        self.origin = [0, 0, JOINT_Z_OFFSET]  # location of leg origin with respect to robot origin
        if hip_flip:
            self.origin[0] = -JOINT_X_OFFSET
        else:
            self.origin[0] = JOINT_X_OFFSET
        if flipped:
            self.origin[1] = -JOINT_Y_OFFSET
        else:
            self.origin[1] = JOINT_Y_OFFSET

    def __repr__(self):
        return "Hip_Lat Pin: %s , Hip_Long Pin: %s , Knee Pin: %s , Flipped: %s" % (
            self.hip_lat.channel, self.hip_long.channel, self.knee.channel, self.flipped)

    def set_position(self, x: float, y: float, z: float):
        """
        A function used to set the position of the foot. The position is relative to the point where the longitudinal
        hip joint and upper leg meet.

        :argument x:type int: The position of the foot in the forward - backward direction in millimeters
        :argument y:type int: The position of the foot in the up - down direction in millimeters
        :argument z:type int: The position of the foot in the in - out direction in millimeters
        :return: None

        References:
        https://www.youtube.com/watch?v=4rc8N1xuWvc&t=2s&ab_channel=AdvancedHobbyLab
        """
        self.current_position = [x, y, z]
        # calculate geometry used in angle calculations
        d = sqrt((z ** 2 + y ** 2) - HIP_LENGTH ** 2)  # distance from hip lat joint to the foot
        g = sqrt(d ** 2 + x ** 2)  # distance from hip long joint to the foot
        # calculate all three joint angles using inverse kinematics
        lat_hip_angle = atan2(z, y) + math.atan2(d, HIP_LENGTH)

        knee_arg = (g ** 2 - UPPER_LEG_LENGTH ** 2 - LOWER_LEG_LENGTH ** 2) / (-2 * UPPER_LEG_LENGTH * LOWER_LEG_LENGTH)
        # Clamp to valid range
        knee_arg = max(-1.0, min(1.0, knee_arg))
        knee_angle = acos(knee_arg)
        try:
            long_hip_angle = atan2(x, d) + asin((LOWER_LEG_LENGTH * sin(knee_angle)) / g)
        except ZeroDivisionError:
            long_hip_angle = 0
        # set all three servos to the calculated angles
        self.hip_lat.set_angle(-1 * (90 - math.degrees(lat_hip_angle)))
        self.hip_long.set_angle(-math.degrees(long_hip_angle))
        self.knee.set_angle(180 - math.degrees(knee_angle))
        return None

    def move(self, dx: float, dy: float, dz: float):
        """
        A function to change, or move, the position of the foot. The given position is relative to the current position.

        :argument dx:type float: the distance in millimeters to change the x position by
        :argument dy:type float: the distance in millimeters to change the y position by
        :argument dz:type float:  the distance in millimeters to change the z position by
        :return: None
        """
        # set the position of the leg to the current position plus the changes given as arguments
        self.set_position(self.current_position[0] + dx, self.current_position[1] + dy, self.current_position[2] + dz)
        return None

    def smooth_move(self, dx: float, dy: float, dz: float, duration: float = 0.75):
        """
        A function to change, or move, the position of the foot smoothly to prevent choppy movements. The given position
        is relative to the current position. Uses servo easing for smoother motion

        :param duration:type float: the time in seconds that the step will execute
        :argument dx:type float: the distance in millimeters to change the x position by
        :argument dy:type float: the distance in millimeters to change the y position by
        :argument dz:type float: the distance in millimeters to change the z position by
        :return: None
        """
        # Don't waste time and processing moving nowhere
        if dx == 0 and dy == 0 and dz == 0:
            return None

        # Number of frames: ~30 Hz
        fps = 30
        steps = int(duration * fps)  # number of steps that will occur
        start = tuple(self.current_position)  # current position is the starting position of the servo.
        end = (start[0] + dx, start[1] + dy, start[2] + dz)  # end position is the start plus the necessary change

        # default easing method is ease out circular method
        easer = Ease_out_quart(start=0, end=1, duration=1)

        for frame in range(steps + 1):
            alpha = frame / steps  # in [0..1]
            t = easer(alpha)  # eased alpha
            # interpolate each axis
            x = start[0] + (end[0] - start[0]) * t
            y = start[1] + (end[1] - start[1]) * t
            z = start[2] + (end[2] - start[2]) * t
            self.set_position(x, y, z)
            time.sleep(duration / steps)
        return None


class CrateJaws:
    """
    The CrateJaws class is used to control the jaws that hold the crate within the body of Speck. This system is made of
    two linear actuators that can both move either forwards or backwards.

    :parameter self.FrontActuator:type Motor: the actuator in the front of Speck
    :parameter self.BackActuator:type Motor: the actuator in the back of Speck
    """

    def __init__(self):
        """
        Constructor for the Crate Jaws class
        """
        # create objects to control the two linear actuators that make up the Crate Jaws. Each linear actuator has two
        # control pins. Front: IN1 and IN2, Back: IN3 and IN4
        self.IN1 = OutputDevice(PIN_IN1, initial_value=False)
        self.IN2 = OutputDevice(PIN_IN2, initial_value=False)
        self.IN3 = OutputDevice(PIN_IN3, initial_value=False)
        self.IN4 = OutputDevice(PIN_IN4, initial_value=False)

    def open(self):
        """
        Function used to open the jaws

        :return: None
        """
        # start moving both linear actuators backwards
        self.IN1.off()
        self.IN2.on()
        self.IN3.on()
        self.IN4.off()
        # create timer object to allow a pause to happen in the background
        timer = Timer(JAW_OPEN_TIME, self.stop)
        # start the timer so that the linear actuators stop after the given amount of time
        timer.start()
        return None

    def close(self):
        """
        Function used to close the jaws

        :return: None
        """
        # start moving both linear actuators forwards
        self.IN1.on()
        self.IN2.off()
        self.IN3.off()
        self.IN4.on()
        # create timer object to allow a pause to happen in the background
        timer = Timer(JAW_OPEN_TIME, self.stop)
        # start the timer so that the linear actuators stop after the given amount of time
        timer.start()
        return None

    def stop(self):
        """
        Function used to stop both linear actuators

        :return: None
        """
        self.IN1.off()
        self.IN2.off()
        self.IN3.off()
        self.IN4.off()
        return None


class Camera:
    """
    The Camera class is used to represent the pi camera module used for detecting the orientation of the crate
    """

    def __init__(self):
        """
        Constructor for the Camera class    
        """
        # define directory to store images
        self.directory = "Images"
        # make sure this directory exists
        os.makedirs(self.directory, exist_ok=True)
        # define and start camera object
        self.camera = Picamera2()
        self.camera.start()
        # keep track of most recent image
        self.most_recent_image = ""
        # define LED for flash
        self.LED = LED(PIN_FLASH)

    def take_picture(self):
        """
        Function used to take a picture and keep track of where this most recent image is stored.

        :return: None
        """
        # pull LED pin low to turn on and wait one second
        self.LED.off()
        time.sleep(1)
        # define image path and name
        path = f"{self.directory}/Raw Image - {datetime.datetime.now()}.jpg"
        # take picture
        self.camera.capture_file(path)
        # Confirm file was written
        if os.path.exists(path):
            print(f"Image saved successfully at {path}")
        else:
            print(f"Image NOT saved at {path}")
        # update most recent path to store the path to the image just taken
        self.most_recent_image = path
        # pull LED pin high to turn off
        self.LED.on()

    def process_image(self, image, blur: int, sensitivity: float, loops=0):
        """
        A recursive function used to detect the crate and determine how to adjust to center the crate.

        :param image: The image to process
        :param blur: The amount of blur used to reduce noise in the image
        :param sensitivity: The sensitivity of the edge detection algorithm
        :param loops: The number of times the image has been processed
        :return: Found, shift_x, shift_y, twist: a flag that tells Speck if the crate was found and the amount of
        correction necessary to align the crate. X is the front and back direction and y is the side to side direction.
        Twist is the amount of rotation necessary to align the crate.
        """
        if loops > 100:
            print("!!Failed to Find Crate!!")
            return False, 0, 0, 0

        # Apply a Gaussian blur to reduce noise
        blurred_image = cv2.GaussianBlur(image, (blur, blur), 0)

        # create copy of image to draw on
        image_copy = image.copy()

        # Convert image to grayscale for edge detection
        img_gray = cv2.cvtColor(blurred_image, cv2.COLOR_BGR2GRAY)

        # Calculate limits for edge detection using the grayscale image
        median = np.mean(blurred_image)
        lower = int(max(0, (1.0 - sensitivity) * median))
        upper = int(min(255, (1.0 + sensitivity) * median))

        # Detect edges followed by 1 iteration of dilation and erosion to remove any background noise.
        edge_image = cv2.Canny(img_gray, lower, upper)
        edge_image = cv2.dilate(edge_image, None, iterations=1)
        edge_image = cv2.erode(edge_image, None, iterations=1)

        # Find contours
        contours, hierarchy = cv2.findContours(edge_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        # Define arrays of important objects
        large_contours = []
        squares = []
        center_points = []

        # Detect squares and draw them on the image
        for c in contours:
            # filter out small contours
            if cv2.contourArea(c) < 25:
                continue  # skip small contours
            large_contours.append(c)

            # find bounding box around each contour
            rect = cv2.minAreaRect(c)
            box = cv2.boxPoints(rect)
            box = np.array(box, dtype='int')

            # Get the width and height from the rectangle
            (w, h) = rect[1]
            if h == 0 or w == 0:
                continue  # avoid division by zero

            # Compute the aspect ratio (ensure it's >= 1 for logic to work)
            aspect_ratio = float(w) / h if w >= h else float(h) / w

            # contour is a square if aspect ratio is within 10% of 1
            is_square = aspect_ratio <= 1.1

            # if it is a square
            if is_square:
                # add to list of squares
                squares.append(rect)

                # Calculate centroid
                cX, cY = np.array(np.mean(box, axis=0), dtype='int')
                center_points.append([cX, cY])

                # draw contours on original image in yellow to improve recursion performance
                cv2.drawContours(image, c, -1, (0, 255, 255), 1)

                # draw the bounding box, center point, and corner circles
                cv2.drawContours(image_copy, [box], -1, (255, 0, 0), 2)
                cv2.drawContours(image_copy, [box], -1, (255, 0, 0), 2)
                cv2.circle(image_copy, (cX, cY), 2, (0, 0, 255), 3)
                for (x, y) in list(box):
                    # print('(x,y):',(x,y))
                    cv2.circle(image_copy, (x, y), 2, (255, 0, 0), 2)

        # draw point in center of image
        image_center = [int(image.shape[1] / 2), int(image.shape[0] / 2)]
        cv2.circle(image_copy, image_center, 2, (0, 255, 0), 4)

        # define adjustment variables. Variables will be updated in later stage
        shift_x = 0
        shift_y = 0
        twist = 0

        # convert list of center points and squares to an array
        center_points = np.array(center_points)

        # determine orientation of box based on number of squares detected
        if len(squares) < 3:  # didn't find any squares
            print('Found %i sqaures(s)' % len(squares))
            if 9 >= blur > 1:  # reprocess image with less blur
                return self.process_image(image, blur - 2, sensitivity, loops + 1)
            elif blur == 1 and sensitivity < 1:  # reprocess image with more sensitivity
                return self.process_image(image, 9, sensitivity + 0.05, loops + 1)
            elif sensitivity >= 1:
                return self.process_image(image, 9, sensitivity - .05, loops + 1)
            crate_center = None

        elif len(squares) == 3:
            print('Three Squares Found')
            # use center points of squares to construct a box
            (x, y), rad = cv2.minEnclosingCircle(center_points)
            # if square one is significantly larger than the others
            if (squares[0][1][0] > 1.25 * squares[1][1][0]) and (squares[0][1][0] > 1.25 * squares[2][1][0]):
                # assume that it contains the entire crate.
                crate_center = [int(squares[0][0][0]), int(squares[0][0][1])]
                rect = squares[0]
            # if square two is significantly larger than the others
            elif (squares[1][1][0] > 1.25 * squares[0][1][0]) and (squares[1][1][0] > 1.25 * squares[2][1][0]):
                # assume that is contains the entire crate.
                crate_center = [int(squares[1][0][0]), int(squares[1][0][1])]
                rect = squares[1]
            # if square two is significantly larger than the others
            elif (squares[2][1][0] > 1.25 * squares[0][1][0]) and (squares[2][1][0] > 1.25 * squares[1][1][0]):
                # assume that is contains the entire crate.
                crate_center = [int(squares[2][0][0]), int(squares[2][0][1])]
                rect = squares[2]
            # otherwise assume three corners were found
            else:
                crate_center = [int(x), int(y)]
                cv2.circle(image_copy, crate_center, int(rad), (0, 0, 255), 2)
                rect = cv2.minAreaRect(center_points)
            box = cv2.boxPoints(rect)
            box = np.array(box, dtype='int')
            cv2.drawContours(image_copy, [box], -1, (0, 0, 255), 2)
            # calculate adjustments
            side_length = np.max(rect[1])
            shift_x = (image_center[0] - crate_center[0]) * CRATE_WIDTH / side_length
            shift_y = (image_center[1] - crate_center[1]) * CRATE_WIDTH / side_length
            twist = rect[-1] % 90
            # only need to rotate angles less than 45 degrees
            if twist > 45:
                twist = twist - 90
        elif len(squares) == 4:  # found 3 - 4 squares, assume 3-4 corners found
            print('Four Squares Found')
            # use center points of squares to construct a box
            rect = cv2.minAreaRect(center_points)
            box = np.array(cv2.boxPoints(rect), dtype=int)
            # use side length of rectangle as reference for length
            side_length = np.max(rect[1])
            # find center of bounding box
            crate_center = [int(rect[0][0]), int(rect[0][1])]
            cv2.drawContours(image_copy, [box], -1, (0, 0, 255), 2)
            # calculate adjustments
            shift_x = (image_center[0] - crate_center[0]) * CRATE_WIDTH / side_length
            shift_y = (image_center[1] - crate_center[1]) * CRATE_WIDTH / side_length
            # find angle to the nearest 90 degrees
            twist = rect[-1] % 90
            # only need to rotate angles less than 45 degrees
            if twist > 45:
                twist = twist - 90
        else:
            print('More than 4 squares found, could not find crate')
            return False, 0, 0, 0

        # draw center point
        cv2.circle(image_copy, crate_center, 2, (0, 0, 255), 4)
        # draw array from point center to image center
        cv2.arrowedLine(image_copy, crate_center, image_center, (0, 0, 0), 1)

        # draw adjustments onto image
        cv2.putText(image_copy,  # image on which to draw text
                    'Shift X = %.4f mm' % shift_x,
                    (10, 20),  # bottom left corner of text
                    cv2.FONT_HERSHEY_SIMPLEX,  # font to use
                    0.5,  # font scale
                    (255, 0, 0),  # color
                    1,  # line thickness
                    )
        cv2.putText(image_copy,  # image on which to draw text
                    'Shift Y = %.4f mm' % shift_y,
                    (10, 40),  # bottom left corner of text
                    cv2.FONT_HERSHEY_SIMPLEX,  # font to use
                    0.5,  # font scale
                    (255, 0, 0),  # color
                    1,  # line thickness
                    )
        cv2.putText(image_copy,  # image on which to draw text
                    'Twist = %.4f deg' % twist,
                    (10, 60),  # bottom left corner of text
                    cv2.FONT_HERSHEY_SIMPLEX,  # font to use
                    0.5,  # font scale
                    (255, 0, 0),  # color
                    1,  # line thickness
                    )

        print('twist:', twist, "deg")
        print('shift X:', shift_x, "mm")
        print('Shift Y:', shift_y, "mm")

        # Save and Return Results
        path = f"{self.directory}/Processed Image - {datetime.datetime.now()}.jpg"
        if cv2.imwrite(path, image_copy):
            print(f"Image saved to {path}")
        else:
            print("Failed to save image.")
        return True, shift_x, shift_y, twist

    def detect_crate(self):
        """
        Function used to take a picture and detect if a crate is present and how to shift to properly align the crate.

        :return: shift_x, shift_y, twist: the amount of correction necessary to align the crate. X is the front and back
        direction and y is the side to side direction. Twist is the amount of rotation necessary to align the crate.
        """
        self.take_picture()
        # read the image from most recent image
        image = cv2.imread(self.most_recent_image)
        # call recursive function to detect crate
        found, shift_x, shift_y, twist = self.process_image(image, 9, 0.2)
        return found, shift_x, shift_y, twist


# __________Main Speck Object__________
class Speck:
    """
    The Speck class is used to control the primary functions of Speck including motion, object detection, and crate
    grabbing.

    :parameter self.rf_leg:type Leg: a Leg object representing the right front leg of Speck
    :parameter self.lf_leg:type Leg:a Leg object representing the left front leg of Speck
    :parameter self.rb_leg:type Leg: a Leg object representing the right back leg of Speck
    :parameter self.lb_leg:type Leg: a Leg object representing the left back leg of Speck
    :parameter self.ObjectSensors:type Array: An array of 5 ObjectDetector objects to create a 175 degree envelope of
    object detection in front of Speck
    :parameter self.LimitSwitches:type Array: An array of 2 Button objects that represent the limit switches used to
    detect a crate within the body of Speck
    :parameter self.CrateJaws:type CrateJaws: an object representing the jaws used to hold the crate within the body
    of Speck
    :parameter self.Gaits:type array: an array of possible gaits that Speck can perform. Each Gait is an array of leg
    movements. The available Gaits are [0]: Walk, [1]: Strafe [2]:Left Turn, [3]:Right Turn.
    """

    def __init__(self):
        """
        Constructor for the Speck Class. Used to initialize Speck
        """
        # create an array of four leg objects
        # [RF, LF, RB, LB]
        self.Legs = [Leg(PIN_RF_HIP_LAT, PIN_RF_HIP_LONG, PIN_RF_KNEE, hip_flip=True),
                     Leg(PIN_LF_HIP_LAT, PIN_LF_HIP_LONG, PIN_LF_KNEE, flipped=True, hip_flip=True),
                     Leg(PIN_RB_HIP_LAT, PIN_RB_HIP_LONG, PIN_RB_KNEE),
                     Leg(PIN_LB_HIP_LAT, PIN_LB_HIP_LONG, PIN_LB_KNEE, flipped=True)]

        # create an array of 5 button objects to represent the object detectors.
        self.ObjectSensors = [Button(PIN_FAR_LEFT_SENSOR), Button(PIN_LEFT_SENSOR), Button(PIN_CENTER_SENSOR),
                              Button(PIN_RIGHT_SENSOR),
                              Button(PIN_FAR_RIGHT_SENSOR)]

        # create an array of buttons to control the limit switches
        self.LimitSwitches = [Button(PIN_LEFT_SWITCH), Button(PIN_RIGHT_SWITCH)]
        # set function for switches to perform when pressed
        for button in self.LimitSwitches:
            button.hold_time = 0.5
            button.when_held = lambda: self.CrateJaws.close() if self.LimitSwitches[0].is_active and self.LimitSwitches[
                1].is_active else print("Switch Held")

        # create the Crate Jaws object used for holding onto the crate
        self.CrateJaws = CrateJaws()
        self.CrateJaws.open()  # make sure the crate jaws start open

        # create the camera object used for detecting the crate
        try:
            self.Camera = Camera()
        except RuntimeError as e:
            print(f"Camera Not Available: {e}")

        # create state flags
        self.is_standing = False

        # create an array of the available gaits
        self.Gaits = [WALK_GAIT]
        # create a queue of movements for each leg to perform. Start with an infinite size
        #                  [RF_Queue, LF_Queue, RB_Queue, LB_Queue]
        self.move_queues = [Queue(0), Queue(0), Queue(0), Queue(0)]
        # Initialize pigpio for improved servo control
        subprocess.run(['sudo', 'systemctl', 'enable', 'pigpiod'])
        # create movement threads to allow motion of each leg to be controlled in the background
        RF_move_thread = Thread(target=self.leg_thread_function, daemon=True, args=(0,))
        LF_move_thread = Thread(target=self.leg_thread_function, daemon=True, args=(1,))
        RB_move_thread = Thread(target=self.leg_thread_function, daemon=True, args=(2,))
        LB_move_thread = Thread(target=self.leg_thread_function, daemon=True, args=(3,))
        self.move_threads = [RF_move_thread, LF_move_thread, RB_move_thread, LB_move_thread]
        # start all movement threads running in the background
        self.lock = Lock()  # Prevents simultaneous uncoordinated movements
        self.thread_barrier = Barrier(4)  # Ensures 4 threads synchronize
        for thread in self.move_threads:
            thread.start()
        # start Speck in sitting position
        self.set_sit()

        # Start bluetooth thread
        Bluetooth_thread = Thread(target=self.bluetooth_server(), daemon=False)
        Bluetooth_thread.start()

        # Store the version of code
        self.Version = "0.0.1"

    # __________Define Movement Thread Function_________
    def leg_thread_function(self, leg_id):
        """
        Function to continuously check the movement queue for movement commands for this leg. This function runs in a
        separate thread for each leg to allow all legs to move at once. Barriers are implemented to synchronize leg
        motion.
        :param leg_id: The ID of the leg to move.
        :return: None
        """
        while True:  # create infinite loop to continue checking for commands in the movement queue and execute them
            self.thread_barrier.wait()  # wait for all threads to be ready, prevents threads from getting ahead,
            # only one loop is performed at a time
            move = self.move_queues[leg_id].get(block=True) # get the next movement in the queue when one is available
            try:
                duration = move[4]
            except IndexError:
                duration = 0.75
            if move[0] == 4:  # if command is for all legs
                self.thread_barrier.wait()  # wait for all threads to be ready
                self.Legs[leg_id].smooth_move(move[1], move[2], move[3], duration)  # move the leg
            elif move[0] == leg_id:  # if command is target at this leg
                self.Legs[leg_id].smooth_move(move[1], move[2], move[3], duration)  # move the leg
            else:  # not for this leg, do nothing
                pass

    # __________Bluetooth Server Function__________
    def bluetooth_server(self):
        """
        Function used to set up bluetooth connection and continuously check for commands. This function runs in a
        separate thread so that Speck is always looking for new commands.
        :return: None
        """
        pass

    # __________Define Speck's Functions__________
    def check_collision(self):
        """
        Function used to check all object sensors for a possible collision

        :return: collision:type boolean: flag to detect if any of the sensors are active. This indicates if a collision
        is possible
        :return: collision_array:type array[5]: an array of boolean values that indicate which sensors are active.
        Sensors are in order from left to right from Speck's perspective
        """
        collision_array = np.zeros(5)  # create an array to store what sensors are triggering a collision
        collision = False  # initialize collision flag to False
        for i, sensor in enumerate(self.ObjectSensors):
            collision_array[i] = sensor.is_active
            if collision_array[i]:  # if any sensor is active, flip the collision flag
                collision = True
        return collision, collision_array

    def set_stand(self):
        """
        Function used to make Speck quickly stand. Sets the position of all feet accordingly
        :return: None
        """
        self.is_standing = True
        self.Legs[1].set_position(-25, 175, HIP_LENGTH)
        self.Legs[0].set_position(-25, 175, HIP_LENGTH)
        self.Legs[3].set_position(25, 175, HIP_LENGTH)
        self.Legs[2].set_position(25, 175, HIP_LENGTH)
        return None

    def stand(self):
        """
        Function used to make Speck slowly stand. Sets the position of all feet accordingly
        :return: None
        """
        self.is_standing = False
        for i in range(0, 2, 1):
            self.move_queues[i].put([4, 25 - self.Legs[i].current_position[0], 175 - self.Legs[i].current_position[1],
                                     HIP_LENGTH - self.Legs[i].current_position[2], 1.5])
        for i in range(2, 4, 1):
            self.move_queues[i].put([4, 25 - self.Legs[i].current_position[0], 175 - self.Legs[i].current_position[1],
                                     HIP_LENGTH - self.Legs[i].current_position[2], 1.5])

    def set_sit(self):
        """
        Function used to make Speck quickly sit. Sets the position of all feet accordingly
        :return: None
        """
        self.is_standing = False
        self.Legs[0].set_position(18, 40, HIP_LENGTH)
        self.Legs[1].set_position(18, 40, HIP_LENGTH)
        self.Legs[2].set_position(18, 40, HIP_LENGTH)
        self.Legs[3].set_position(18, 40, HIP_LENGTH)
        return None

    def sit(self):
        """
        Function used to make Speck slowly sit. Sets the position of all feet accordingly
        :return: None
        """
        self.is_standing = False
        for i in range(0, 4, 1):
            self.move_queues[i].put([4, 18 - self.Legs[i].current_position[0], 40 - self.Legs[i].current_position[1],
                                     HIP_LENGTH - self.Legs[i].current_position[2], 2.0])
        return None

    def gait(self, gait):
        """
        function to run basic gait motions without smooth motion. Passed a sequence of movement arrays.

        :param gait:type: int[[int, int, int, int]]: an array of movement arrays. One movement array in the form
        [[Legs], dx, dy, dz] where Leg is the leg to move (0 = RF, 1 = LF, 2 = RB, 3 = LB, 4 = ALL). dx dy and dz and
        changes in the foot position in millimeters
        :return: None
        """
        self.is_standing = False
        # Gait Layout:
        # {Step n: {Leg, dx, dy, dz},
        # {Step n+1: {Leg, dx, dy, dz}}
        for step in range(0, len(gait), 1):  # loop through all steps for one cycle
            for leg in range(4):  # add movement to all four move queues,
                # if the command is not meant for one leg, nothing will happen
                if leg in gait[step][0]:
                    self.move_queues[leg].put([leg, gait[step][1], gait[step][2], gait[step][3]])
                elif 4 in gait[step][0]:
                    self.move_queues[leg].put([4, gait[step][1], gait[step][2], gait[step][3]])

    def walk(self, steps):
        """
        Function used to walk forward a set number of steps
        :param steps: number of steps to take
        :return: None
        """
        if not self.is_standing:  # if Speck isn't standing
            self.stand()
            time.sleep(2)  # wait 2 second before walking
        for step in range(steps):
            self.gait(self.Gaits[0])

    def reverse(self, steps):
        """
        Function used to walk forward a set number of steps
        :param steps: number of steps to take
        :return: None
        """
        if not self.is_standing:  # if Speck isn't standing
            self.stand()
            time.sleep(1)  # wait 1 second before walking
        for step in range(steps):
            self.gait(self.Gaits[1])

    def strafe_left(self):
        """
        Function used to strafe or side step to the left
        :return: None
        """
        if not self.is_standing:  # if Speck isn't standing
            self.stand()
        self.gait(self.Gaits[2])

    def strafe_right(self):
        """
        Function used to strafe or side step to the right
        :return: None
        """
        if not self.is_standing:  # if Speck isn't standing
            self.stand()
        self.gait(self.Gaits[3])

    def turn_left(self):
        """
        Function used to turn left
        :return: None
        """
        if not self.is_standing:  # if Speck isn't standing
            self.stand()
        self.gait(self.Gaits[4])

    def turn_right(self):
        """
        Function used to turn right
        :return: None
        """
        if not self.is_standing:  # if Speck isn't standing
            self.stand()
        self.gait(self.Gaits[5])

    def shift(self, forward: bool, distance: int):
        """
        Function used to slightly shift the robot for proper crate alignment.

        :param forward: boolean value to choose direction of shift. True will shift front and back. False will shift
        side to side
        :param distance: the distance to shift in mm.
        :return: None
        """
        if forward:
            for leg in range(4):  # add movement to all four move queues,
                # if the command is not meant for one leg, nothing will happen
                self.move_queues[leg].put([4, distance, 0, 0])
        else:
            for leg in range(4):  # add movement to all four move queues,
                # if the command is not meant for one leg, nothing will happen
                self.move_queues[leg].put([4, 0, 0, ((-1) ** (leg % 2)) * distance])
        return None

    def rotate(self, pitch: float, roll: float, yaw: float, center_of_rotation=[0.0, 0.0, 0.0], duration=0.5):
        """
        Function used to rotate Speck in place by tilting or turning the body. This simulates a rotation of the body
        relative to the ground by adjusting the leg positions accordingly.

        :param pitch: Rotation about the local X-axis (tilting forward/backward), in degrees  → Global Z
        :param roll:  Rotation about the local Z-axis (tilting left/right), in degrees       → Global X
        :param yaw:   Rotation about the local Y-axis (twisting left/right), in degrees      → Global Y
        :param center_of_rotation: The point (in global coordinates) to rotate about
        :param duration: The time in seconds for each leg to move to its new position
        :return: None

        Axis Mapping:
            Local  →  Global
             X     →     Z
             Y     →     X
             Z     →     Y
        """
        # ---- MAX LIMITS ----
        MAX_PITCH = 15  # deg forward/back tilt
        MAX_ROLL = 15  # deg left/right tilt
        MAX_YAW = 30  # deg in-place twist

        # Clamp and convert to radians (applied to GLOBAL axes)
        pitch = np.radians(max(-MAX_PITCH, min(MAX_PITCH, pitch)))  # Local X → Global Z
        roll = np.radians(max(-MAX_ROLL, min(MAX_ROLL, roll)))  # Local Z → Global X
        yaw = np.radians(max(-MAX_YAW, min(MAX_YAW, yaw)))  # Local Y → Global Y

        # Map to GLOBAL frame as pitch=Z, roll=X, yaw=Y
        cz, cx, cy = np.cos([pitch, roll, yaw])
        sz, sx, sy = np.sin([pitch, roll, yaw])

        # Rotation matrix: R = Ryaw @ Rpitch @ Rroll → R = R_y @ R_z @ R_x
        R = np.array([
            [cy * cz, cy * sz * sx - sy * cx, cy * sz * cx + sy * sx],
            [sy * cz, sy * sz * sx + cy * cx, sy * sz * cx - cy * sx],
            [-sz, cz * sx, cz * cx]
        ])

        center = np.array(center_of_rotation)
        deltas = [[0], [0], [0], [0]]

        for leg_num, leg in enumerate(self.Legs):
            origin = np.array(leg.origin)
            current = np.array(leg.current_position)
            global_foot = origin + current

            # Rotate foot about the body center
            rotated = R @ (global_foot - center) + center
            new_local = rotated - origin
            delta = new_local - current

            print(f"Leg {leg_num} change in foot position: {delta}")

            # Flip x (forward/back) if leg is mirrored
            delta[2] = -delta[2] if leg.flipped else delta[2]

            # Output as [Z, Y, X] to match your local [X, Y, Z]
            deltas[leg_num] = [leg_num, delta[0], delta[1], delta[2], duration]

        for leg_num in range(4):
            self.move_queues[leg_num].put(deltas[leg_num])

    def grab(self):
        """
        Function used to manually grab a crate by closing the crate jaws.
        :return: None
        """
        print("Grabbing")
        self.CrateJaws.close()
        return None

    def drop(self):
        """
        Function used to drop a crate by manually opening the crate jaws.
        :return: None
        """
        print("Dropping")
        self.CrateJaws.open()  # open jaws
        return None

    def center_crate(self):
        """
        Function that takes a picture, detects shift and twist necessary to center crate, and makes the necessary
        adjustments
        :return: None
        """
        # use camera object to find shift and twist
        found, shiftx, shifty, twist = self.Camera.detect_crate()
        while (abs(shiftx) > 150) or (abs(shifty) > 150):
            print("Shift too large")
            found, shiftx, shifty, twist = self.Camera.detect_crate()
        if found:
            if shiftx >= 7 or shiftx <= 3:
                self.shift(forward=True, distance=-shiftx)
            if shifty >= 2 or shifty <= -2:
                self.shift(forward=False, distance=-shifty)
            if twist > 10:
                self.twist(cw=True, theta=twist)
            elif twist < -10:
                self.twist(cw=False, theta=twist)
        return None

    def grab_crate(self):
        """
        This function centers the crate and grabs it if it is close enough to the center
        :return: None
        """
        self.center_crate()
        found, x, y, theta = self.Camera.detect_crate()
        while (abs(x) > 150) or (abs(y) > 150):
            found, x, y, theta = self.Camera.detect_crate()
        if found and (3 < x < 7) and (-2 < y < 2) and (-10 < theta < 10):
            self.sit()
        else:
            self.center_crate()
            found, x, y, theta = self.Camera.detect_crate()
            while (abs(x) > 150) or (abs(y) > 150):
                found, x, y, theta = self.Camera.detect_crate()
            if found and (3 < x < 7) and (-2 < y < 2) and (-10 < theta < 10):
                self.sit()
            else:
                print("Couldn't Center Crate")

    def __repr__(self):
        return "RF Leg Position: %i, %i, %i" \
               "LF Leg Position: %i, %i, %i" \
               "RB Leg Position: %i, %i, %i" \
               "LB Leg Position: %i, %i, %i" % \
            (self.Legs[0].current_position[0], self.Legs[0].current_position[1], self.Legs[0].current_position[2],
             self.Legs[1].current_position[0], self.Legs[1].current_position[1], self.Legs[1].current_position[2],
             self.Legs[2].current_position[0], self.Legs[2].current_position[1], self.Legs[2].current_position[2],
             self.Legs[3].current_position[0], self.Legs[3].current_position[1], self.Legs[3].current_position[2],)
