"""
Ryan Poliquin, Started 2/25/2025
This code is used to set the angles of all leg servos so legs can be properly installed
"""
from Speck import Speck

speck = Speck()
for leg in speck.Legs:
    leg.knee.set_angle(90)
    leg.hip_long.set_angle(90)
    leg.hip_lat.set_angle(0)
