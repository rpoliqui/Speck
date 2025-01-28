"""
Ryan Poliquin, started on 1/27/2025
This code contains all definitions required to control Speck. It is broken into the basic objects that
need to be controlled. These classes are combined into the top level class, Speck.
_______________________________________________________________________________________________________________________
Speck: The object representing Speck as a whole. All commands should be sent to this object and handled by the
       corresponding objects
Joint: A single servo joint. Defined by the minimum and maximum the angles can move to and the starting angle of the joint
Leg: A combination of several joints that form a leg
"""


class Joint:
    """
    The Joint class is used to represent a single joint in a leg assembly.

    :param self.min_angle: the minimum angle that the joint can be set to
    :param self.max_angle: the maximum angle that the joint can be set to
    :param self.current_angle: the angle that the joint is currently at
    """

    def __init__(self, min_angle=0, max_angle=180, starting_angle=0):
        """
        Constructor for the Joint class.
        
        :param min_angle: the minimum angle that the joint can be set to
        :param max_angle: the maximum angle that the joint can be set to
        :param starting_angle: the angle to set the joint to on startup
        """
        self.min_angle = min_angle  # the minimum angle that the joint can move to
        self.max_angle = max_angle  # the maximum angle that the joint can move to
        self.current_angle = starting_angle  # the current angle of the joint which starts at the starting angle
        self.set_angle(starting_angle)  # properly set the starting angle of the joint

    def set_angle(self, angle):
        """
        A function used to set the angle of the joint

        :param angle: the angle to set the joint to
        :return: None
        """
        # check to make sure the requested angle is within the range of the joint
        if angle <= self.max_angle & angle >= self.min_angle:
            self.current_angle = angle  # update the current angle of the joint to the required angle
        else:
            pass
        return None

    def change_angle(self, change_in_angle):
        """
        A function used to change the angle of the joint

        :param change_in_angle: the amount to change the angle of the joint
        :return: None
        """
        # set the angle of the joint to the current angle plus the change
        self.set_angle(self.current_angle + change_in_angle)
        return None


class Leg:
    """
    The Leg class is used to represent a single leg of Speck.

    :param self.hip_lat: a Joint object representing the tilt of the leg from the body
    :param self.hip_long: a Joint object representing the main hip joint
    :param self.knee: a Joint object representing the knee joint
    """
    def __init__(self):
        """
        Constructor for the Leg class.
        """
        self.hip_lat = Joint(starting_angle=0)
        self.hip_long = Joint(starting_angle=0)
        self.knee = Joint(starting_angle=0)

    def set_position(self, x, y, z):
        """
        A function used to set the position of the foot. The position is relative to ___

        :param x:
        :param y:
        :param z:
        :return: None
        """
        return None

    pass


class Speck:
    """
    The Speck class is used to control the primary functions of Speck including motion, object detection, and crate grabbing.

    :param self.rf_leg: a Leg object representing the right front leg of Speck
    :param self.lf_leg: a Leg object representing the left front leg of Speck
    :param self.rb_leg: a Leg object representing the right back leg of Speck
    :param self.lb_leg: a Leg object representing the left back leg of Speck
    """
    def __int__(self):
        self.rf_leg = Leg()
        self.lf_leg = Leg()
        self.rb_leg = Leg()
        self.lb_leg = Leg()

    def step(self):
        pass
