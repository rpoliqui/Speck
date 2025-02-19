"""
Ryan Poliquin, started on 1/27/2025
This code contains all definitions required to control Speck. It is broken into the basic objects that
need to be controlled. These classes are combined into the top level class, Speck. All pins are defined at the beginning
of the code so that they may be modified for different implementations.
_______________________________________________________________________________________________________________________
Classes:
    Speck: The object representing Speck as a whole. All commands should be sent to this object and handled by the
           corresponding objects
    Joint: A single servo joint. Defined by the minimum and maximum the angles can move to and the starting angle of the joint
    Leg: A combination of three joints that form a leg
    ObjectDetector: A single infrared avoidance sensor used to get information about Speck's surroundings
Global Variables:
    AvailablePins: a boolean array used to keep track of what pins are available. True = available, False = unavailable
_______________________________________________________________________________________________________________________
References:
    Python. (2025, January 8). Python 3.13.1 documentation. Retrieved from Python: https://docs.python.org/3/
    Geeks for Geeks. (2024, August 2). Python Docstrings. Retrieved from Geeks for Geeks: https://www.geeksforgeeks.org/python-docstrings/
    https://forums.raspberrypi.com/viewtopic.php?t=173157
    https://stackoverflow.com/questions/8247605/configuring-so-that-pip-install-can-work-from-github
    https://gpiozero.readthedocs.io/en/latest/
    https://gitpython.readthedocs.io/en/stable/tutorial.html
    https://packaging.python.org/en/latest/tutorials/packaging-projects/
    https://projects.raspberrypi.org/en/projects/getting-started-with-git/0
    https://stackoverflow.com/questions/66054625/pyinstaller-error-running-script-with-pyzmq-dependency
    https://git-scm.com/docs/git-pull
    https://docs.python.org/3/library/math.html
"""
# __________Import Statements__________
import numpy as np
from gpiozero import AngularServo
import subprocess
import os
import math
from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import Device

# __________Pin Definition__________
# Joint Pins
PIN_LF_HIP_LAT = 1
PIN_RF_HIP_LAT = 2
PIN_LB_HIP_LAT = 3
PIN_RB_HIP_LAT = 4
PIN_LF_HIP_LONG = 5
PIN_RF_HIP_LONG = 6
PIN_LB_HIP_LONG = 7
PIN_RB_HIP_LONG = 8
PIN_LF_KNEE = 9
PIN_RF_KNEE = 10
PIN_LB_KNEE = 11
PIN_RB_KNEE = 12

# Motor Driver Pins

# Object Sensor Pins

# Limit Switch

# __________System Constants__________
HIP_LENGTH = 10
UPPER_LEG_LENGTH = 117
LOWER_LEG_LENGTH = 125

# __________Global Variables__________
# Create an array of boolean values to keep track of what GPIO pins are available on the pi
# True = available; False = unavailable
AvailablePins = np.ones(40)

# __________Environment Setup__________
factory = PiGPIOFactory()  # define pin factory to use servos for more accurate servo control


# __________Class Definitions__________
class Joint:
    """
    The Joint class is used to represent a single joint in a leg assembly.

    :param self.pin:type int: the GPIO pin that the joint servo is connected to
    :param self.min_angle: int: the minimum angle that the joint can be set to
    :param self.max_angle: int: the maximum angle that the joint can be set to
    :param self.current_angle: int: the angle that the joint is currently at
    """

    def __init__(self, pin: int, min_angle=0.0, max_angle=180.0, starting_angle=0.0):
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
        self.min_angle = min_angle  # define min angle
        self.max_angle = max_angle  # define max angle
        self.current_angle = starting_angle  # set the starting angle
        # create servo object to control physical servo object
        self.servo = AngularServo(self.pin, min_angle=self.min_angle, max_angle=self.max_angle, pin_factory=factory)
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

    :parameter self.hip_lat:type Joint: a Joint object representing the tilt of the leg from the body, known as the lateral
    hip joint
    :parameter self.hip_long:type Joint: a Joint object representing the main hip joint, known as the longitudinal hip joint
    :parameter self.knee:type Joint: a Joint object representing the knee joint
    :parameter self.current_position:type {int, int, int}: an array with the current position of the foot in the form
    {x, y, z} in millimeters.
    """

    def __init__(self, hip_lat_pin: int, hip_long_pin: int, knee_pin: int):
        """
        Constructor for the Leg class.

        :argument hip_lat_pin:type int: The pin that the lateral hip joint servo is connected to
        :argument hip_long_pin:type int: The pin that the longitudinal hip joint servo is connected to
        :argument knee_pin:type int: The pin that the knee joint is connected to
        """
        # check to make sure all given pins are available. Raise an error if the pin is unavailable. Set the pins to taken
        if AvailablePins[hip_lat_pin - 1] == 1:
            self.hip_lat = Joint(hip_lat_pin, starting_angle=0)
            AvailablePins[hip_lat_pin - 1] = 0
        else:
            raise RuntimeError("Pin " + str(hip_lat_pin) + " is not available to use for the lateral hip joint.")

        if AvailablePins[hip_long_pin - 1] == 1:
            self.hip_long = Joint(hip_long_pin, starting_angle=0)
            AvailablePins[hip_long_pin - 1] = 0
        else:
            raise RuntimeError("Pin " + str(hip_long_pin) + " is not available to use for the longitudinal hip joint.")

        if AvailablePins[knee_pin - 1] == 1:
            self.knee = Joint(knee_pin, starting_angle=0)
            AvailablePins[knee_pin - 1] = 0
        else:
            raise RuntimeError("Pin " + str(knee_pin) + " is not available for the knee joint")
        self.current_position = [0, 0, 0]  # assume robot starts at origin until the position of the joints is set.

    def set_position(self, x: int, y: int, z: int):
        """
        A function used to set the position of the foot. The position is relative to the point where the longitudinal
        hip joint and upper leg meet.

        :argument x:type int: The position of the foot in the forward - backward direction in millimeters
        :argument y:type int: The position of the foot in the up - down direction in millimeters
        :argument z:type int: The position of the foot in the in - out direction in millimeters
        :return: None
        """
        self.current_position = {x, y, z}  # update the parameter storing the current position
        # calculate all three joint angles using inverse kinematics
        lat_hip_angle = math.atan(z / y) + math.atan(math.sqrt(z ** 2 + y ** 2 - HIP_LENGTH ** 2) / HIP_LENGTH)
        knee_angle = math.acos(
            (z ** 2 + y ** 2 - HIP_LENGTH ** 2 + x ** 2 - UPPER_LEG_LENGTH ** 2 - LOWER_LEG_LENGTH ** 2) / (
                    -2 * UPPER_LEG_LENGTH * LOWER_LEG_LENGTH))
        long_hip_angle = math.atan(x / (math.sqrt(z ** 2 + y ** 2 - HIP_LENGTH ** 2))) + math.asin(
            (LOWER_LEG_LENGTH * math.sin(knee_angle)) / (math.sqrt(z ** 2 + y ** 2 - HIP_LENGTH ** 2 + x ** 2)))
        # set all three servos to the calculated angles
        self.hip_lat.set_angle(math.degrees(lat_hip_angle))
        self.hip_long.set_angle(math.degrees(long_hip_angle))
        self.knee.set_angle(math.degrees(knee_angle))
        return None

    def move(self, dx, dy, dz):
        """
        A function to change, or move, the position of the foot. The given position is relative to the current position.

        :argument dx:type int: the distance in millimeters to change the x position by
        :argument dy:type int: the distance in millimeters to change the y position by
        :argument dz:type int:  the distance in millimeters to change the z position by
        :return: None
        """
        self.set_position(self.current_position[0] + dx, self.current_position[1] + dy, self.current_position[2] + dz)
        return None

    pass


class ObjectDetector:
    """
    The ObjectDectecor class is used to represent a single infrared avoidance sensor.

    :param pin: int: the GPIO pin that the sensor is connected to
    """

    def __init__(self, pin: int):
        self.pin = pin
        self.value = int


class Speck:
    """
    The Speck class is used to control the primary functions of Speck including motion, object detection, and crate grabbing.

    :param self.rf_leg: Leg: a Leg object representing the right front leg of Speck
    :param self.lf_leg: Leg:a Leg object representing the left front leg of Speck
    :param self.rb_leg: Leg: a Leg object representing the right back leg of Speck
    :param self.lb_leg: Leg: a Leg object representing the left back leg of Speck
    """

    def __init__(self):
        """
        Constructor for the Speck Class. Used to initialize Speck
        """
        #
        self.rf_leg = Leg(PIN_RF_HIP_LAT, PIN_RF_HIP_LONG, PIN_RF_KNEE)
        self.lf_leg = Leg(PIN_LF_HIP_LAT, PIN_LF_HIP_LONG, PIN_LF_KNEE)
        self.rb_leg = Leg(PIN_RB_HIP_LAT, PIN_RB_HIP_LONG, PIN_RB_KNEE)
        self.lb_leg = Leg(PIN_LB_HIP_LAT, PIN_LB_HIP_LONG, PIN_LB_KNEE)
        self.ObjectSensors = {ObjectDetector(13), ObjectDetector(14), ObjectDetector(15)}

    def step(self):
        pass

    def stand(self):
        """
        Function used to set Speck
        """
        self.lf_leg.set_position(0, 100, 0)
        self.rf_leg.set_position(0, 100, 0)
        self.lb_leg.set_position(0, 100, 0)
        self.rb_leg.set_position(0, 100, 0)

    def sit(self):
        self.lf_leg.set_position(0, 0, 0)
        self.rf_leg.set_position(0, 0, 0)
        self.lb_leg.set_position(0, 0, 0)
        self.rb_leg.set_position(0, 0, 0)

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
                    "\n\n_____________________________________________________________________________________________________")
                print("Installing Speck from GitHub\n")
                if os.path.isdir(os.path.join(target_dir, '.git')):
                    # Pull the latest changes from GitHub and merge changes
                    subprocess.run(['git', '-C', target_dir, 'pull', 'origin', 'main', '--ff-only'])
                else:
                    # If the repository doesn't exist, clone the repository from GitHub
                    subprocess.run(['git', 'clone', 'https://github.com/rpoliqui/Speck/', target_dir])
            if (scope == "ALL"):
                print(
                    "\n\n_____________________________________________________________________________________________________")
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
