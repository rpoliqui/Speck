"""
Ryan Poliquin, started 2/26/2025
Used to test that legs were installed correctly by sweeping through basic leg functions
"""
from Speck import Speck
import time

speck = Speck()  # create instance of Speck to control
for leg in speck.Legs:
    # test hip tilt mobility
    print("__________Testing Hip Tilt Mobility__________")
    leg.hip_lat.set_angle(0)  # set to starting angle
    time.sleep(0.5)
    print("Upper Range:")
    for angle in range(0, 20, 1):  # sweep through upper range
        leg.hip_lat.set_angle(angle)
        print(angle)
        time.sleep(0.5)
    print("Lower Range:")
    for angle in range(0, -20, -1):  # sweep through lower range
        leg.hip_lat.set_angle(angle)
        print(angle)
        time.sleep(0.5)
    leg.hip_lat.set_angle(0)  # reset to starting angle
    time.sleep(0.5)

    # test hip longitudinal mobility
    print("__________Testing Hip Longitudinal Mobility__________")
    leg.hip_lat.set_angle(0)  # set to starting angle
    time.sleep(0.5)
    print("Upper Range:")
    for angle in range(0, 90, 1):  # sweep through upper range
        leg.hip_lat.set_angle(angle)
        print(angle)
        time.sleep(0.5)
    print("Lower Range:")
    for angle in range(0, -90, -1):  # sweep through lower range
        leg.hip_lat.set_angle(angle)
        print(angle)
        time.sleep(0.5)
    leg.hip_lat.set_angle(0)  # reset to starting angle
    time.sleep(0.5)

    # test knee mobility
    print("__________Testing Knee Mobility__________")
    leg.hip_lat.set_angle(90)  # set to starting angle
    time.sleep(0.5)
    print("Upper Range:")
    for angle in range(90, 180, 1):  # sweep through upper range
        leg.hip_lat.set_angle(angle)
        print(angle)
        time.sleep(0.5)
    print("Lower Range:")
    for angle in range(90, 0, -1):  # sweep through lower range
        leg.hip_lat.set_angle(angle)
        print(angle)
        time.sleep(0.5)
    leg.hip_lat.set_angle(90)  # reset to starting angle
    time.sleep(0.5)

    # test x direction movement
    print("__________Testing X Direction Movement__________")
    leg.set_position(0, 100, 34)  # set to starting position
    time.sleep(0.5)
    print("Forward")
    leg.move(100, 0, 0)  # move forward
    time.sleep(1)
    leg.move(-100, 0, 0)  # move back to start
    time.sleep(1)
    print("Backward")
    leg.move(-100, 0, 0)  # move backward
    time.sleep(1)

    # test y direction movement
    print("__________Testing Y Direction Movement__________")
    leg.set_position(0, 100, 34)  # set to starting position
    time.sleep(0.5)
    print("Forward")
    leg.move(0, 100, 0)  # move forward
    time.sleep(1)
    leg.move(0, -100, 0)  # move back to start
    time.sleep(1)
    print("Backward")
    leg.move(0, -100, 0)  # move backward
    time.sleep(1)

    # test z direction movement
    print("__________Testing Z Direction Movement__________")
    leg.set_position(0, 100, 34)  # set to starting position
    time.sleep(1)
    print("Forward")
    leg.move(0, 0, 50)  # move forward
    time.sleep(1)
    leg.move(0, 0, -50)  # move back to start
    time.sleep(1)
    print("Backward")
    leg.move(0, 0, -50)  # move backward
    time.sleep(1)
