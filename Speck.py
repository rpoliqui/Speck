"""
Ryan Poliquin, started on 1/27/2025
This code contains all definitions required to control Speck. It is broken into the basic objects that
need to be controlled. These classes are combined into the top level class, Speck.
_______________________________________________________________________________________________________________________
Classes:
    Speck: The object representing Speck as a whole. All commands should be sent to this object and handled by the
           corresponding objects
    Joint: A single servo joint. Defined by the minimum and maximum the angles can move to and the starting angle of the joint
    Leg: A combination of several joints that form a leg
    ObjectDetector: A single infrared avoidance sensor used to get information about Speck's surroundings
Global Variables:
    AvailablePins: a boolean array used to keep track of what pins are available. True = available, False = unavailable
_______________________________________________________________________________________________________________________
References:
    Python. (2025, January 8). Python 3.13.1 documentation. Retrieved from Python: https://docs.python.org/3/
    Geeks for Geeks. (2024, August 2). Python Docstrings. Retrieved from Geeks for Geeks: https://www.geeksforgeeks.org/python-docstrings/
"""
# __________Import Statements__________
import numpy as np

# __________Global Variables__________
# Create an array of boolean values to keep track of what GPIO pins are available on the pi
# True = available; False = unavailable
AvailablePins = np.array(40)
for i in AvailablePins: # set all pins as to true to start
    AvailablePins[i] = True

# __________Class Definitions__________
class Joint:
    """
    The Joint class is used to represent a single joint in a leg assembly.

    :param self.pin: int: the GPIO pin that the joint servo is connected to
    :param self.min_angle: int: the minimum angle that the joint can be set to
    :param self.max_angle: int: the maximum angle that the joint can be set to
    :param self.current_angle: int: the angle that the joint is currently at
    """

    def __init__(self, pin:int, min_angle=0, max_angle=180, starting_angle=0):
        """
        Constructor for the Joint class.
        
        :param pin: int: the GPIO pin that the joint servo is connected to
        :param min_angle: int: the minimum angle that the joint can be set to
        :param max_angle: int: the maximum angle that the joint can be set to
        :param starting_angle: int: the angle to set the joint to on startup
        """
        if AvailablePins[pin]:  # If the pin is available, set it up and mark it as used
            self.pin = pin
            AvailablePins[pin] = False

        # TODO: setup pin for analog PWM output
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.current_angle = starting_angle
        self.set_angle(starting_angle)  # properly set the starting angle of the joint

    def set_angle(self, angle:int):
        """
        A function used to set the angle of the joint

        :param angle: int: the angle to set the joint to
        :return: None
        """
        # check to make sure the requested angle is within the range of the joint
        if angle <= self.max_angle & angle >= self.min_angle:
            self.current_angle = angle  # update the current angle of the joint to the required angle
        else:
            raise RuntimeError("The given angle was out of the joint's range " + str(self.min_angle) + )
            pass
        return None

    def change_angle(self, change_in_angle:int):
        """
        A function used to change the angle of the joint

        :param change_in_angle: int: the amount to change the angle of the joint
        :return: None
        """
        # set the angle of the joint to the current angle plus the change
        self.set_angle(self.current_angle + change_in_angle)
        return None


class Leg:
    """
    The Leg class is used to represent a single leg of Speck.

    :param self.hip_lat: Joint: a Joint object representing the tilt of the leg from the body
    :param self.hip_long: Joint: a Joint object representing the main hip joint
    :param self.knee: Joint: a Joint object representing the knee joint
    """

    def __init__(self, hip_lat_pin:int, hip_long_pin:int, knee_pin:int):
        """
        Constructor for the Leg class.
        """
        self.hip_lat = Joint(hip_lat_pin, starting_angle=0)
        self.hip_long = Joint(hip_long_pin, starting_angle=0)
        self.knee = Joint(knee_pin, starting_angle=0)

    def set_position(self, x, y, z):
        """
        A function used to set the position of the foot. The position is relative to ___

        :param x:
        :param y:
        :param z:
        :return: None
        """
        return None

    def move(self, x, y, z):
        """
        A function to change, or move, the position of the foot. The given position is relative to the current position.

        :param x:
        :param y:
        :param z:
        :return: None
        """
        return None

    pass


class ObjectDetector:
    """
    The ObjectDectecor class is used to represent a single infrared avoidance sensor
    """

    def __init__(self, pin:int):
        self.pin = pin
        self.value = int


class Speck:
    """
    The Speck class is used to control the primary functions of Speck including motion, object detection, and crate grabbing.

    :param self.rf_leg: a Leg object representing the right front leg of Speck
    :param self.lf_leg: a Leg object representing the left front leg of Speck
    :param self.rb_leg: a Leg object representing the right back leg of Speck
    :param self.lb_leg: a Leg object representing the left back leg of Speck
    """

    def __init__(self):
        """
        Constructor for the Speck Class.
        """
        self.rf_leg = Leg(1,2,3)
        self.lf_leg = Leg(4,5,6)
        self.rb_leg = Leg(7,8,9)
        self.lb_leg = Leg(10,11,12)
        self.ObjectSensors = {ObjectDetector(13), ObjectDetector(14), ObjectDetector(15)}

    def step(self):
        pass

    def update(self):
        successful = False

        return successful
