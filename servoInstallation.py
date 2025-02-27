"""
Ryan Poliquin, Started 2/25/2025
This code is used to set the angles of all leg servos so legs can be properly installed
"""
from Speck import Speck

speck = Speck()
legs = ["RF", "LF", "RB", "LB"]
legNum = 0
for leg in speck.Legs:
    print(legs[legNum])
    print(leg)
    leg.knee.set_angle(0)
    leg.hip_long.set_angle(0)
    leg.hip_lat.set_angle(0)
    legNum += 1
