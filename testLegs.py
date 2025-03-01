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
    leg.hip_long.set_angle(0)  # set to starting angle
    time.sleep(0.5)
    print("Upper Range:")
    for angle in range(0, 90, 5):  # sweep through upper range
        leg.hip_long.set_angle(angle)
        print(angle)
        time.sleep(0.5)
    print("Lower Range:")
    for angle in range(0, -90, -5):  # sweep through lower range
        leg.hip_long.set_angle(angle)
        print(angle)
        time.sleep(0.5)
    leg.hip_long.set_angle(0)  # reset to starting angle
    time.sleep(0.5)

    # test knee mobility
    print("__________Testing Knee Mobility__________")
    leg.knee.set_angle(90)  # set to starting angle
    time.sleep(0.5)
    print("Upper Range:")
    for angle in range(90, 180, 5):  # sweep through upper range
        leg.knee.set_angle(angle)
        print(angle)
        time.sleep(0.5)
    print("Lower Range:")
    for angle in range(90, 0, -5):  # sweep through lower range
        leg.knee.set_angle(angle)
        print(angle)
        time.sleep(0.5)
    leg.knee.set_angle(90)  # reset to starting angle
    time.sleep(0.5)

    # test x direction movement
    print("__________Testing X Direction Movement__________")
    leg.set_position(0, 150, 62)  # set to starting position
    time.sleep(2)
    print("Forward")
    leg.move(-50, 0, 0)  # move forward
    time.sleep(1)
    leg.move(50, 0, 0)  # move back to start
    time.sleep(1)
    print("Backward")
    leg.move(50, 0, 0)  # move backward
    time.sleep(1)

    # test y direction movement
    print("__________Testing Y Direction Movement__________")
    leg.set_position(0, 150, 62)  # set to starting position
    time.sleep(2)
    print("Forward")
    leg.move(0, 50, 0)  # move forward
    time.sleep(1)
    leg.move(0, -50, 0)  # move back to start
    time.sleep(1)
    print("Backward")
    leg.move(0, -50, 0)  # move backward
    time.sleep(1)

    # test z direction movement
    print("__________Testing Z Direction Movement__________")
    leg.set_position(0, 150, 62)  # set to starting position
    time.sleep(2)
    print("Forward")
    leg.move(0, 0, 20)  # move forward
    time.sleep(1)
    leg.move(0, 0, -20)  # move back to start
    time.sleep(1)
    print("Backward")
    leg.move(0, 0, -20)  # move backward
    time.sleep(1)

    # test multi-axis movement
    print("__________Test Multi-Axis Movement__________")
    leg.set_position(0, 150, 62)  # set to starting position
    time.sleep(2)
    print("X, Y Movement")
    leg.move(50, 50, 0)
    time.sleep(1)
    leg.move(-50, -50, 0)
    time.sleep(1)
    print("X, Z Movement")
    leg.move(50, 0, 20)
    time.sleep(1)
    leg.move(-50, 0, -20)
    time.sleep(1)
    print("Y, Z Movement")
    leg.move(0, 50, 20)
    time.sleep(1)
    leg.move(0, -50, -20)
    time.sleep(1)
    print("X, Y, Z Movement")
    leg.move(50, 50, 20)
    time.sleep(1)
    leg.move(-50, -50, -20)
    time.sleep(1)

print("\nFinished Legs\n")
# test preset positions and gaits
print("__________Testing Preset Positions and Movements__________")
print("Sitting")
speck.sit()
time.sleep(2)
print("Standing")
speck.stand()
time.sleep(2)
print("Walking")
speck.gait(speck.Gaits[0])
