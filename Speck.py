"""
Ryan Poliquin, started on 1/27/2025
This script contains all definitions required to control the Speck robot. It is broken into the basic objects that
need to be controlled.
_______________________________________________________________________________________________________________________
Joint: a single servo joint. Defined by the minimum and maximum the angles can move to and the starting angle of the joint
Leg: a combination of several joints that form a leg
Speck: the object representing the robot as a whole. All commands should be sent to this object and handled by the
       corresponding objects
"""


class Joint:
    def __init__(self, minAngle=0, maxAngle=180, startingAngle=0):
        self.minAngle = minAngle
        self.maxAngle = maxAngle
        self.currentAngle = startingAngle
        self.set_angle(startingAngle)

    def set_angle(self, angle):
        self.currentAngle = angle

    def change_angle(self, change_in_angle):
        self.set_angle(self.currentAngle + change_in_angle)


class Leg:
    def __init__(self):
        self.hip_lat = Joint(startingAngle=0)
        self.hip_long = Joint(startingAngle=0)
        self.knee = Joint(startingAngle=0)

    def set_position(self, X, Y, Z):
        pass

    pass


class Speck:
    def __int__(self):
        self.rf_leg = Leg()
        self.lf_leg = Leg()
        self.rb_leg = Leg()
        self.lb_leg = Leg()

    def step(self):
        pass
