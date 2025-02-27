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
    ObjectDetector: A single infrared avoidance sensor used to get information about Speck's surroundings
    Camera: An object that controls a Raspberry Pi Camera module and performs all operations to detect the crate
    CrateJaws: An object used to control the crate holding jaws that are used to pick up a specially designed crate
Global Variables:
    AvailablePins: a boolean array used to keep track of what pins are available. True = available, False = unavailable
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
"""
# __________Import Statements__________
import numpy as np
import subprocess
import os
import math
import time
# import cv2 as cv
# from picamera import PiCamera
from threading import Thread, Timer
from gpiozero import AngularServo, Motor, Button, Device
from gpiozero.pins.pigpio import PiGPIOFactory

# __________Pin Definition__________
# Joint Pins
PIN_LF_HIP_LAT = 10
PIN_RF_HIP_LAT = 17
PIN_LB_HIP_LAT = 25
PIN_RB_HIP_LAT = 18
PIN_LF_HIP_LONG = 9
PIN_RF_HIP_LONG = 27
PIN_LB_HIP_LONG = 8
PIN_RB_HIP_LONG = 23
PIN_LF_KNEE = 11
PIN_RF_KNEE = 22
PIN_LB_KNEE = 7
PIN_RB_KNEE = 24

# Motor Driver Pins
PIN_IN1 = 14
PIN_IN2 = 15
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
HIP_LENGTH = 34
UPPER_LEG_LENGTH = 123.75
LOWER_LEG_LENGTH = 110
JAW_OPEN_TIME = 10
JAW_CLOSE_TIME = 10

# __________Global Variables__________
# Create an array of boolean values to keep track of what GPIO pins are available on the pi
# 1 = available; 0 = unavailable
AvailablePins = np.ones(40)

# array storing changes in x, y and z positions for each leg to enable Speck to walk. Layout:
# {Step n: { RF: {x, y, z}, LF: {x, y, z}, RB: {x, y, z}, LB: {x, y, z}},
#  Step n+1: { RF: {x, y, z}, LF: {x, y, z}, RB: {x, y, z}, LB: {x, y, z}}}
# WALK_GAIT = {{{x, y, z}, {x, y, x}, {x, y, z}, {x, y, z}},
#              {{x, y, z}, {x, y, x}, {x, y, z}, {x, y, z}},
#              {{x, y, z}, {x, y, x}, {x, y, z}, {x, y, z}},
#              {{x, y, z}, {x, y, x}, {x, y, z}, {x, y, z}},
#              {{x, y, z}, {x, y, x}, {x, y, z}, {x, y, z}},
#              {{x, y, z}, {x, y, x}, {x, y, z}, {x, y, z}}}

# array storing changes in x, y and z positions for each leg to enable Speck to walk. Layout:
# {Step n: {Leg, dx, dy, dz},
# {Step n+1: {Leg, dx, dy, dz}}
# LEGS: [RF, LF, RB, LB] 4 = ALL
# DIRECTIONS: [X, Y, Z]
WALK_GAIT = ((0, 0, 50, 0),
             (0, 50, 0, 0),
             (0, 0, -50, 0),
             (3, 0, 50, 0),
             (3, 50, 0, 0),
             (3, 0, -50, 0),
             (4, -50, 0, 0),
             (1, 0, 50, 0),
             (1, 50, 0, 0),
             (1, 0, -50, 0),
             (2, 0, 50, 0),
             (2, 50, 0, 0),
             (2, 0, -50, 0),
             (4, -50, 0, 0))

# __________Environment Setup__________
factory = PiGPIOFactory()  # define pin factory to use servos for more accurate servo control
Device.pin_factory = factory
background = Thread()  # create a background Thread to allow processes to run in the background
background.start()  # start the background Thread


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
            raise RuntimeError("The given angle was out of the joint's range " + str(self.min_angle))
            pass
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

    def __init__(self, hip_lat_pin: int, hip_long_pin: int, knee_pin: int, flipped=False):
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
            self.hip_lat = Joint(hip_lat_pin, min_angle=-90, max_angle=90, starting_angle=0, flipped=not flipped)
            AvailablePins[hip_lat_pin - 1] = 0
        else:
            raise RuntimeError("Pin " + str(hip_lat_pin) + " is not available to use for the lateral hip joint.")

        if AvailablePins[hip_long_pin - 1] == 1:
            self.hip_long = Joint(hip_long_pin, min_angle=-90, max_angle=90, starting_angle=0, flipped=flipped)
            AvailablePins[hip_long_pin - 1] = 0
        else:
            raise RuntimeError("Pin " + str(hip_long_pin) + " is not available to use for the longitudinal hip joint.")

        if AvailablePins[knee_pin - 1] == 1:
            self.knee = Joint(knee_pin, min_angle=0, max_angle=180, starting_angle=0, flipped=flipped)
            AvailablePins[knee_pin - 1] = 0
        else:
            raise RuntimeError("Pin " + str(knee_pin) + " is not available for the knee joint")
        self.flipped = flipped  # specify whether all angles need to be flipped or not
        self.current_position = [0, 0, 0]  # assume robot starts at origin until the position of the joints is set.

    def __repr__(self):
        return "Hip_Lat Pin: %s , Hip_Long Pin: %s , Knee Pin: %s , Flipped: %s" % (
            self.hip_lat.pin, self.hip_long.pin, self.knee.pin, self.flipped)

    def set_position(self, x: int, y: int, z: int):
        """
        A function used to set the position of the foot. The position is relative to the point where the longitudinal
        hip joint and upper leg meet.

        :argument x:type int: The position of the foot in the forward - backward direction in millimeters
        :argument y:type int: The position of the foot in the up - down direction in millimeters
        :argument z:type int: The position of the foot in the in - out direction in millimeters
        :return: None
        """
        self.current_position = [x, y, z]  # update the parameter storing the current position
        # calculate all three joint angles using inverse kinematics
        lat_hip_angle = math.atan(z / y) + math.atan(math.sqrt(z ** 2 + y ** 2 - HIP_LENGTH ** 2) / HIP_LENGTH)
        knee_angle = math.acos(
            (z ** 2 + y ** 2 - HIP_LENGTH ** 2 + x ** 2 - UPPER_LEG_LENGTH ** 2 - LOWER_LEG_LENGTH ** 2) / (
                    -2 * UPPER_LEG_LENGTH * LOWER_LEG_LENGTH))
        long_hip_angle = math.atan(x / (math.sqrt(z ** 2 + y ** 2 - HIP_LENGTH ** 2))) + math.asin(
            (LOWER_LEG_LENGTH * math.sin(knee_angle)) / (math.sqrt(z ** 2 + y ** 2 - HIP_LENGTH ** 2 + x ** 2)))
        # set all three servos to the calculated angles
        self.hip_lat.set_angle(90-math.degrees(lat_hip_angle))
        self.hip_long.set_angle(90-math.degrees(long_hip_angle))
        self.knee.set_angle(180-math.degrees(knee_angle))
        return None

    def move(self, dx, dy, dz):
        """
        A function to change, or move, the position of the foot. The given position is relative to the current position.

        :argument dx:type int: the distance in millimeters to change the x position by
        :argument dy:type int: the distance in millimeters to change the y position by
        :argument dz:type int:  the distance in millimeters to change the z position by
        :return: None
        """
        # set the position of the leg to the current position plus the changes given as arguments
        self.set_position(self.current_position[0] + dx, self.current_position[1] + dy, self.current_position[2] + dz)
        return None


class ObjectDetector:
    """
    The ObjectDectecor class is used to represent a single infrared avoidance sensor.

    :parameter self.pin:type int: the GPIO pin that the sensor is connected to
    :parameter self.value:type int: the value of the sensor
    """

    def __init__(self, pin: int):
        self.pin = pin
        self.value = int


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
        # create objects to control the two linear actuators that make up the Crate Jaws. Each linear actuator is a
        # motor connected to two pins on the motor driver
        self.FrontActuator = Motor(PIN_IN1, PIN_IN2)
        self.BackActuator = Motor(PIN_IN3, PIN_IN4)
        # make sure both actuators are stopped when initialized
        self.FrontActuator.stop()
        self.BackActuator.stop()

    def open(self):
        """
        Function used to open the jaws

        :return: None
        """
        # start moving both linear actuators backwards
        self.FrontActuator.backward()
        self.BackActuator.backward()
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
        self.FrontActuator.forward()
        self.BackActuator.forward()
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
        self.BackActuator.stop()
        self.FrontActuator.stop()
        return None


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
    """

    def __init__(self):
        """
        Constructor for the Speck Class. Used to initialize Speck
        """
        # create an array of four leg objects
        # [RF, LF, RB, LB]
        self.Legs = [Leg(PIN_RF_HIP_LAT, PIN_RF_HIP_LONG, PIN_RF_KNEE),
                     Leg(PIN_LF_HIP_LAT, PIN_LF_HIP_LONG, PIN_LF_KNEE, flipped=True),
                     Leg(PIN_RB_HIP_LAT, PIN_RB_HIP_LONG, PIN_RB_KNEE),
                     Leg(PIN_LB_HIP_LAT, PIN_LB_HIP_LONG, PIN_LB_KNEE, flipped=True)]
        # create an array of 5 object detection sensors
        self.ObjectSensors = [ObjectDetector(PIN_FAR_LEFT_SENSOR), ObjectDetector(PIN_LEFT_SENSOR),
                              ObjectDetector(PIN_CENTER_SENSOR), ObjectDetector(PIN_RIGHT_SENSOR),
                              ObjectDetector(PIN_FAR_RIGHT_SENSOR)]
        # create an array of buttons to control the limit switches
        self.LimitSwitches = [Button(PIN_LEFT_SWITCH), Button(PIN_RIGHT_SWITCH)]
        # create the Crate Jaws object used for holding onto the crate
        self.CrateJaws = CrateJaws()
        self.CrateJaws.open()  # make sure the crate jaws start open
        # create the camera object used for detecting the crate
        self.Camera = Camera()
        # create an array of the available gaits
        self.Gaits = [WALK_GAIT]

    def step(self):
        pass

    def stand(self):
        """
        Function used to make Speck stand. Sets the position of all feet accordingly
        """
        self.Legs[0].set_position(0, 200, 34)
        self.Legs[1].set_position(0, 200, 34)
        self.Legs[2].set_position(0, 200, 34)
        self.Legs[3].set_position(0, 200, 34)

    def sit(self):
        """
        Function used to make Speck sit. Sets the position of all feet accordingly
        """
        self.Legs[0].set_position(0, 50, 34)
        self.Legs[1].set_position(0, 50, 34)
        self.Legs[2].set_position(0, 50, 34)
        self.Legs[3].set_position(0, 50, 34)

    def gait(self, gait):
        # Gait Layout:
        # {Step n: {Leg, dx, dy, dz},
        # {Step n+1: {Leg, dx, dy, dz}}
        for step in range(0, len(gait) - 1, 1):  # loop through all steps for once cyle
            if gait[step][0] == 4:  # move all legs
                for leg in self.Legs:
                    leg.move(gait[step][1], gait[step][2], gait[step][3])
            else:
                self.Legs[gait[step][0]].move(gait[step][1], gait[step][2], gait[step][3])

    def grab(self):
        self.sit()  # have Speck sit onto the crate
        Timer(5, self.CrateJaws.close)  # wait 5 seconds for Speck to sit, then close the jaws
        if (self.LimitSwitches[0].is_active()) & (self.LimitSwitches[1].is_active()):
            Timer(JAW_CLOSE_TIME + 5, self.stand)  # wait for the jaws to close plus a few seconds before standing

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
