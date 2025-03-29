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
    Python. (2025, January 8). Python 3.13.1 documentation. Retrieved from Python: https://docs.python.org/3/
    Geeks for Geeks. (2024, August 2). Python Docstrings. Retrieved from Geeks for Geeks:
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
"""
# __________Import Statements__________
import numpy as np
import subprocess
import os
import math
from math import atan2, sin, asin, acos, sqrt, fabs
import time
# import cv2 as cv
# from picamera import PiCamera
from threading import Thread, Timer
from queue import Queue
from gpiozero import AngularServo, Button, Device, OutputDevice
from gpiozero.pins.pigpio import PiGPIOFactory

# __________Pin Definition__________
# Joint Pins
PIN_RF_HIP_LAT = 14
PIN_RF_HIP_LONG = 15
PIN_RF_KNEE = 18
PIN_LF_HIP_LAT = 17
PIN_LF_HIP_LONG = 27
PIN_LF_KNEE = 22
PIN_RB_HIP_LAT = 23
PIN_RB_HIP_LONG = 24
PIN_RB_KNEE = 25
PIN_LB_HIP_LAT = 10
PIN_LB_HIP_LONG = 9
PIN_LB_KNEE = 11

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

# __________System Constants__________
HIP_LENGTH = 74
UPPER_LEG_LENGTH = 124.5
LOWER_LEG_LENGTH = 110
JAW_OPEN_TIME = 5
JAW_CLOSE_TIME = 5

# __________Global Variables__________
# Create an array of boolean values to keep track of what GPIO pins are available on the pi
# 1 = available; 0 = unavailable
AvailablePins = np.ones(40)

# array storing changes in x, y and z positions for each leg to enable Speck to walk. Layout:
# {Step n: {Leg, dx, dy, dz},
# {Step n+1: {Leg, dx, dy, dz}}
# LEGS: [RF, LF, RB, LB] 4 = ALL
# DIRECTIONS: [X, Y, Z] +X = backwards, +Y = downward
WALK_GAIT = ((0, 0, -50, 0),
             (0, -50, 0, 0),
             (0, 0, 50, 0),
             (3, 0, -50, 0),
             (3, -50, 0, 0),
             (3, 0, 50, 0),
             (4, 50, 0, 0),
             (1, 0, -50, 0),
             (1, -50, 0, 0),
             (1, 0, 50, 0),
             (2, 0, -50, 0),
             (2, -50, 0, 0),
             (2, 0, 50, 0),
             (4, 50, 0, 0))

STRAFE_STEP = 20
STRAFE_GAIT = ((0, 0, -50, 0),
               (0, 0, 0, -STRAFE_STEP),
               (0, 0, 50, 0),
               (3, 0, -50, 0),
               (3, 0, 0, -STRAFE_STEP),
               (3, 0, 50, 0),
               (4, 0, 0, STRAFE_STEP),
               (1, 0, -50, 0),
               (1, 0, 0, -STRAFE_STEP),
               (1, 0, 50, 0),
               (2, 0, -50, 0),
               (2, 0, 0, -STRAFE_STEP),
               (2, 0, 50, 0))

TURN_STEP = 20
LEFT_TURN_GAIT = ((0, 0, -50, 0),
                  (0, 0, 0, TURN_STEP),
                  (0, 0, 50, 0),
                  (3, 0, -50, 0),
                  (3, 0, 0, -TURN_STEP),
                  (3, 0, 50, 0),
                  (1, 0, -50, 0),
                  (1, 0, 0, TURN_STEP),
                  (1, 0, 50, 0),
                  (2, 0, -50, 0),
                  (2, 0, 0, -TURN_STEP),
                  (2, 0, 50, 0),
                  (0, 0, 0, -TURN_STEP),
                  (1, 0, 0, -TURN_STEP),
                  (2, 0, 0, TURN_STEP),
                  (3, 0, 0, TURN_STEP),)

RIGHT_TURN_GAIT = ((0, 0, -50, 0),
                   (0, 0, 0, -TURN_STEP),
                   (0, 0, 50, 0),
                   (3, 0, -50, 0),
                   (3, 0, 0, TURN_STEP),
                   (3, 0, 50, 0),
                   (1, 0, -50, 0),
                   (1, 0, 0, -TURN_STEP),
                   (1, 0, 50, 0),
                   (2, 0, -50, 0),
                   (2, 0, 0, TURN_STEP),
                   (2, 0, 50, 0),
                   (0, 0, 0, TURN_STEP),
                   (1, 0, 0, TURN_STEP),
                   (2, 0, 0, -TURN_STEP),
                   (3, 0, 0, -TURN_STEP),)

# __________Environment Setup__________
factory = PiGPIOFactory()  # define pin factory to use servos for more accurate servo control
Device.pin_factory = factory


# __________Class Definitions__________
class Joint:
    """
    The Joint class is used to represent a single joint in a leg assembly.

    :param self.pin:type int: the GPIO pin that the joint servo is connected to
    :param self.min_angle: int: the minimum angle that the joint can be set to
    :param self.max_angle: int: the maximum angle that the joint can be set to
    :param self.current_angle: int: the angle that the joint is currently at
    """

    def __init__(self, pin: int, min_angle=0.0, max_angle=180.0, starting_angle=0.0, flipped=False):
        """
        Constructor for the Joint class.
        
        :argument pin:type int: the GPIO pin that the joint servo is connected to
        :argument min_angle:type float: the minimum angle that the joint can be set to
        :argument max_angle:type float: the maximum angle that the joint can be set to
        :argument starting_angle:type float: the angle to set the joint to on startup
        """
        if AvailablePins[pin - 1] == 1:  # If the pin is available, set it up and mark it as used
            self.pin = pin
            AvailablePins[pin - 1] = 0
        self.flipped = flipped  # keep track of if all angles need to be flipped
        self.min_angle = min_angle  # define min angle
        self.max_angle = max_angle  # define max angle
        self.current_angle = starting_angle  # set the starting angle
        # create servo object to control physical servo object
        if flipped:  # flip the min and max angles
            self.servo = AngularServo(self.pin, min_angle=self.max_angle, max_angle=self.min_angle,
                                      initial_angle=starting_angle, min_pulse_width=0.0006,
                                      max_pulse_width=0.0025, pin_factory=factory)
        else:
            self.servo = AngularServo(self.pin, min_angle=self.min_angle, max_angle=self.max_angle,
                                      initial_angle=starting_angle, min_pulse_width=0.0006,
                                      max_pulse_width=0.0025, pin_factory=factory)
        self.set_angle(starting_angle)  # properly set the starting angle of the joint

    def set_angle(self, angle: float):
        """
        A function used to set the angle of the joint.

        :argument angle:type float: the angle to set the joint to
        :return: None
        """
        # check to make sure the requested angle is within the range of the joint
        if (angle <= self.max_angle) & (angle >= self.min_angle):
            self.current_angle = angle  # update the current angle of the joint to the required angle
            self.servo.angle = angle  # set the physical angle of the servo
        else:
            raise RuntimeError("The given angle %f was out of the joint's range %f - %f" % (angle, self.min_angle,
                                                                                            self.max_angle))
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

    def __init__(self, hip_lat_pin: int, hip_long_pin: int, knee_pin: int, flipped=False, hip_flip=False):
        """
        Constructor for the Leg class.

        :argument hip_lat_pin:type int: The pin that the lateral hip joint servo is connected to
        :argument hip_long_pin:type int: The pin that the longitudinal hip joint servo is connected to
        :argument knee_pin:type int: The pin that the knee joint is connected to
        """
        # check to make sure all given pins are available. Raise an error if the pin is unavailable.
        # Set the pins to taken
        if AvailablePins[hip_lat_pin - 1] == 1:
            # for correct functioning, the lateral hip joints needs to be flipped by default
            self.hip_lat = Joint(hip_lat_pin, min_angle=-90, max_angle=90, starting_angle=0,
                                 flipped=flipped and hip_flip)
            AvailablePins[hip_lat_pin - 1] = 0
        else:
            raise RuntimeError("Pin " + str(hip_lat_pin) + " is not available to use for the lateral hip joint.")

        if AvailablePins[hip_long_pin - 1] == 1:
            self.hip_long = Joint(hip_long_pin, min_angle=-90, max_angle=90, starting_angle=-90, flipped=flipped)
            AvailablePins[hip_long_pin - 1] = 0
        else:
            raise RuntimeError("Pin " + str(hip_long_pin) + " is not available to use for the longitudinal hip joint.")

        if AvailablePins[knee_pin - 1] == 1:
            self.knee = Joint(knee_pin, min_angle=0, max_angle=180, starting_angle=180, flipped=flipped)
            AvailablePins[knee_pin - 1] = 0
        else:
            raise RuntimeError("Pin " + str(knee_pin) + " is not available for the knee joint")
        self.flipped = flipped  # specify whether all angles need to be flipped or not
        self.current_position = [0, 0, 0]  # assume robot starts at origin until the position of the joints is set.

    def __repr__(self):
        return "Hip_Lat Pin: %s , Hip_Long Pin: %s , Knee Pin: %s , Flipped: %s" % (
            self.hip_lat.pin, self.hip_long.pin, self.knee.pin, self.flipped)

    def set_position(self, x: float, y: float, z: float):
        """
        A function used to set the position of the foot. The position is relative to the point where the longitudinal
        hip joint and upper leg meet.

        :argument x:type int: The position of the foot in the forward - backward direction in millimeters
        :argument y:type int: The position of the foot in the up - down direction in millimeters
        :argument z:type int: The position of the foot in the in - out direction in millimeters
        :return: None
        """
        self.current_position = [x, y, z]
        # calculate geometry used in angle calculations
        d = sqrt((z ** 2 + y ** 2) - HIP_LENGTH ** 2)  # distance from hip lat joint to the foot
        g = sqrt(d ** 2 + x ** 2)  # distance from hip long joint to the foot
        # calculate all three joint angles using inverse kinematics
        lat_hip_angle = atan2(z, y) + math.atan2(d, HIP_LENGTH)

        knee_angle = acos((g ** 2 - UPPER_LEG_LENGTH ** 2 - LOWER_LEG_LENGTH ** 2) /
                          (-2 * UPPER_LEG_LENGTH * LOWER_LEG_LENGTH))
        long_hip_angle = atan2(x, d) + asin((LOWER_LEG_LENGTH * sin(knee_angle)) / g)
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

    def smooth_move(self, dx: float, dy: float, dz: float):
        """
        A function to change, or move, the position of the foot smoothly to prevent choppy movements. The given position
        is relative to the current position.

        :argument dx:type float: the distance in millimeters to change the x position by
        :argument dy:type float: the distance in millimeters to change the y position by
        :argument dz:type float: the distance in millimeters to change the z position by
        :return: None
        """
        # define the number of steps as a quarter of the largest size so that each step is about 4mm. Take absolute
        # value to handle negatives
        step_size = int(max(fabs(dx), fabs(dy), fabs(dz)) / 4)
        for step in range(0, step_size):
            # set the position of the leg to the current position plus the changes given as arguments
            self.set_position(self.current_position[0] + dx / step_size, self.current_position[1] + dy / step_size,
                              self.current_position[2] + dz / step_size)
            time.sleep(.01)
        return None


class Camera:
    """
    The Camera class is used to represent the pi camera module used for detecting the orientation of the crate
    """

    def __init__(self):
        """
        Constructor for the Camera class
        """


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
        self.ObjectSensors = [Button(PIN_FAR_LEFT_SENSOR), Button(PIN_LEFT_SENSOR),Button(PIN_CENTER_SENSOR), Button(PIN_RIGHT_SENSOR),
                              Button(PIN_FAR_RIGHT_SENSOR)]
        # create an array of buttons to control the limit switches
        self.LimitSwitches = [Button(PIN_LEFT_SWITCH), Button(PIN_RIGHT_SWITCH)]
        # set function for switches to perform when pressed
        for button in self.LimitSwitches:
            button.hold_time = 0.5
            button.when_held = lambda: self.grab() if self.LimitSwitches[0].is_active and self.LimitSwitches[1].is_active else print("Switch Held")
        # create the Crate Jaws object used for holding onto the crate
        self.CrateJaws = CrateJaws()
        self.CrateJaws.open()  # make sure the crate jaws start open
        # create the camera object used for detecting the crate
        self.Camera = Camera()
        # create an array of the available gaits
        self.Gaits = [WALK_GAIT, STRAFE_GAIT, LEFT_TURN_GAIT, RIGHT_TURN_GAIT]
        # create a queue of movements for each leg to perform. Start with an infinite size
        #                  [RF_Queue, LF_Queue, RB_Queue, LB_Queue]
        self.move_queues = [Queue(0), Queue(0), Queue(0), Queue(0)]
        # create movement threads to allow motion of each leg to be controlled in the background
        RF_move_thread = Thread(target=self.RF_thread_function, daemon=True)
        LF_move_thread = Thread(target=self.LF_thread_function, daemon=True)
        RB_move_thread = Thread(target=self.RB_thread_function, daemon=True)
        LB_move_thread = Thread(target=self.LB_thread_function, daemon=True)
        self.move_threads = [RF_move_thread, LF_move_thread, RB_move_thread, LB_move_thread]
        # start all movement threads running in the background
        for thread in self.move_threads:
            thread.start()
        # start Speck in sitting position
        self.set_sit()
        # Store the version of code
        self.Version = "0.0.1"

    # __________Define Movement Thread Function_________
    def RF_thread_function(self):
        while True:  # create infinite loop to continue checking for commands in the movement queue and execute them
            if not self.move_queues[0].empty():  # if the queue is not empty
                move = self.move_queues[0].get()  # get the next movement in the queue
                if move[0] == 0:  # if command is target at this leg, move it
                    self.Legs[move[0]].smooth_move(move[1], move[2], move[3])
                else:  # command in wrong queue, move to correct queue
                    self.move_queues[move[0]].put(move)
            else:
                # short delay to wait for next command
                time.sleep(0.5)

    def LF_thread_function(self):
        while True:  # create infinite loop to continue checking for commands in the movement queue and execute them
            if not self.move_queues[1].empty():  # if the queue is not empty
                move = self.move_queues[1].get()  # get the next movement in the queue
                if move[0] == 1:  # if command is target at this leg, move it
                    self.Legs[move[0]].smooth_move(move[1], move[2], move[3])
                else:  # command in wrong queue, move to correct queue
                    self.move_queues[move[0]].put(move)
            else:
                # short delay to wait for next command
                time.sleep(0.5)

    def RB_thread_function(self):
        while True:  # create infinite loop to continue checking for commands in the movement queue and execute them
            if not self.move_queues[2].empty():  # if the queue is not empty
                move = self.move_queues[2].get()  # get the next movement in the queue
                if move[0] == 2:  # if command is target at this leg, move it
                    self.Legs[move[0]].smooth_move(move[1], move[2], move[3])
                else:  # command in wrong queue, move to correct queue
                    self.move_queues[move[0]].put(move)
            else:
                # short delay to wait for next command
                time.sleep(0.5)

    def LB_thread_function(self):
        while True:  # create infinite loop to continue checking for commands in the movement queue and execute them
            if not self.move_queues[3].empty():  # if the queue is not empty
                move = self.move_queues[3].get()  # get the next movement in the queue
                if move[0] == 3:  # if command is target at this leg, move it
                    self.Legs[move[0]].smooth_move(move[1], move[2], move[3])
                else:  # command in wrong queue, move to correct queue
                    self.move_queues[move[0]].put(move)
            else:
                # short delay to wait for next command
                time.sleep(0.5)

    #__________Define Speck's Functions__________
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
        """
        self.Legs[1].set_position(25, 175, HIP_LENGTH)
        self.Legs[0].set_position(25, 175, HIP_LENGTH)
        self.Legs[3].set_position(25, 175, HIP_LENGTH)
        self.Legs[2].set_position(25, 175, HIP_LENGTH)

    def stand(self):
        """
        Function used to make Speck slowly stand. Sets the position of all feet accordingly
        """
        for i in range(3, -1, -1):
            self.move_queues[i].put([i, 25 - self.Legs[i].current_position[0], 175 - self.Legs[i].current_position[1],
                                     HIP_LENGTH - self.Legs[i].current_position[2]])

    def set_sit(self):
        """
        Function used to make Speck quickly sit. Sets the position of all feet accordingly
        """
        self.Legs[0].set_position(20, 50, HIP_LENGTH)
        self.Legs[1].set_position(20, 50, HIP_LENGTH)
        self.Legs[2].set_position(20, 50, HIP_LENGTH)
        self.Legs[3].set_position(20, 50, HIP_LENGTH)

    def sit(self):
        """
        Function used to make Speck slowly sit. Sets the position of all feet accordingly
        """
        for i in range(0, 4, 1):
            self.move_queues[i].put([i, 20 - self.Legs[i].current_position[0], 50 - self.Legs[i].current_position[1],
                                     HIP_LENGTH - self.Legs[i].current_position[2]])

    def gait(self, gait):
        """
        function to run basic gait motions without smooth motion. Passed a sequence of movement arrays.

        :param gait:type: int[[int, int, int, int]]: an array of movement arrays. One movement array in the form
        [Leg, dx, dy, dz] where Leg is the leg to move (0 = RF, 1 = LF, 2 = RB, 3 = LB, 4 = ALL). dx dy and dz and
        changes in the foot position in millimeters
        :return: None
        """
        # Gait Layout:
        # {Step n: {Leg, dx, dy, dz},
        # {Step n+1: {Leg, dx, dy, dz}}
        for step in range(0, len(gait) - 1, 1):  # loop through all steps for one cyCle
            if gait[step][0] == 4:  # move all legs
                for leg in range(4):
                    # add movement to all four move queues
                    self.move_queues[leg].put([leg, gait[step][1], gait[step][2], gait[step][3]])
            else:  # only add the movement to the necessary queue
                self.move_queues[gait[step][0]].put([gait[step][0], gait[step][1], gait[step][2], gait[step][3]])

    def grab(self):
        print("Grabbing")
        if self.LimitSwitches[0].is_active & self.LimitSwitches[1].is_active:
            Timer(2, self.CrateJaws.close)  # wait 2 seconds for Speck to sit, then close the jaws
            Timer(JAW_CLOSE_TIME + 5, self.stand)  # wait for the jaws to close plus a few seconds before standing
        else:
            print("Both Limit Switches not engages, Crate not in correct location")

    def update(self, scope="ESSENTIAL"):
        """
        A function used to pull the most updated Speck code from the GitHub repository and ensure the pi is up-to-date
        and ready to function

        :param scope:type String: Specify the scope of the upgrade. Default value is "ESSENTIAL" which only pulls from
        the GitHub repository. If scope = "ALL", then the raspberry pi is updated, all python packages are updated, and
        all code is pulled from the GitHub repository
        :return:
        """
        successful = False
        subprocess.run(['sudo', 'apt', 'install', '-y', 'git'])  # install git on the pi
        target_dir = os.getcwd()  # define the directory where the repository will be stored
        wifi_ip = subprocess.check_output(['hostname', '-I'])  # get IP address of pi
        if wifi_ip is not None:  # Wi-Fi is connected, so Speck can be updated
            # Check if the repository already exists in the target directory
            if (scope == "ESSENTIAL") | (scope == "ALL"):
                print(
                    "\n\n_____________________________________________________________________________________________")
                print("Installing Speck from GitHub\n")
                if os.path.isdir(os.path.join(target_dir, '.git')):
                    # Pull the latest changes from GitHub and merge changes
                    subprocess.run(['git', '-C', target_dir, 'pull', 'origin', 'main', '--ff-only'])
                else:
                    # If the repository doesn't exist, clone the repository from GitHub
                    subprocess.run(['git', 'clone', 'https://github.com/rpoliqui/Speck/', target_dir])
            if scope == "ALL":
                print(
                    "\n\n_____________________________________________________________________________________________")
                print("Updating Raspberry Pi and All python packages\n")
                version = subprocess.run(['git', 'describe'], capture_output=True, text=True)
                if version != self.Version:
                    print("___________________________________________________________________________________________")
                    print("Current Speck Version: %s. New Speck Version: %s." % (self.Version, version))
                    print("___________________________________________________________________________________________")
                subprocess.run(['sudo', 'apt', '-y', 'update'])  # Update the package list
                subprocess.run(['sudo', 'apt', '-y', 'upgrade'])  # Update the packages
                subprocess.run(['sudo', 'apt', '-y', 'autoremove'])  # Remove any unnecessary packages from the pi
                subprocess.run(['sudo', 'apt', 'install', 'pigpio'])  # Install pigpio for improved pin control
                subprocess.run(['sudo', 'systemctl', 'enable', 'pigpiod'])  # Enable the daemon to run at time of boot
                subprocess.run(['sudo', 'systemctl', 'start', 'pigpiod'])  # Start the daemon now to prevent rebooting
                subprocess.run(['pip', 'install',
                                'pyzmq==21.0.0'])  # Update pyzmq for messages, was throwing error of outdated version
                subprocess.run(['pip', 'install', '--upgrade', 'pip'])  # Update pip
                subprocess.run(['pip', 'install', '--upgrade', 'gpiozero'])  # Update gpiozero
                subprocess.run(['pip', 'install', '--upgrade', 'numpy'])  # Update numpy
            successful = True
        else:  # Wi-Fi is not connected, Speck cannot be updated
            print("Speck cannot be updated without a wifi connection.")
        return successful

    def __repr__(self):
        return "RF Leg Position: %i, %i, %i" \
               "LF Leg Position: %i, %i, %i" \
               "RB Leg Position: %i, %i, %i" \
               "LB Leg Position: %i, %i, %i" % \
            (self.Legs[0].current_position[0], self.Legs[0].current_position[1], self.Legs[0].current_position[2],
             self.Legs[1].current_position[0], self.Legs[1].current_position[1], self.Legs[1].current_position[2],
             self.Legs[2].current_position[0], self.Legs[2].current_position[1], self.Legs[2].current_position[2],
             self.Legs[3].current_position[0], self.Legs[3].current_position[1], self.Legs[3].current_position[2],)
