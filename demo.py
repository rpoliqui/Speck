"""
Script to demonstrate Speck's capabilities
"""
from Speck import Speck
from time import sleep
import math

speck = Speck()
down_time = 1

frequency = 0.2
amplitude = 10

while True:
    # Wait to initialize
    sleep(3)

    # _____Show Standing_____
    speck.stand()
    sleep(down_time)
    dt = 0.02

    # _____Show Pitching_____
    speck.smooth_rotate(15, 0, 0)
    sleep(down_time)
    speck.smooth_rotate(-30, 0, 0)
    sleep(down_time)
    speck.smooth_rotate(15, 0, 0)
    sleep(down_time)

    # _____Show Rolling_____
    speck.smooth_rotate(0, 15, 0)
    sleep(down_time)
    speck.smooth_rotate(0, -30, 0)
    sleep(down_time)
    speck.smooth_rotate(0, 15, 0)
    sleep(down_time)

    # _____Show Circular Motion_____
    omega = 2 * math.pi * frequency
    total_time = 1.0 / frequency  # one full cycle

    # Initial values at t = 0
    t = 0.0
    prev_pitch = amplitude * math.sin(omega * t)
    prev_roll = amplitude * math.cos(omega * t)

    # Loop until one complete gyration is done
    while t < total_time:
        t += dt
        if t > total_time:
            t = total_time  # clamp to finish cleanly

        pitch = amplitude * math.sin(omega * t)
        roll = amplitude * math.cos(omega * t)

        # Incremental changes
        dpitch = pitch - prev_pitch
        droll = roll - prev_roll

        speck.rotate(dpitch, droll, 0)  # yaw = 0, no turning

        prev_pitch = pitch
        prev_roll = roll

        sleep(dt)

    # _____Show Shifting_____
    sleep(down_time)
    speck.shift(True, 10)
    sleep(down_time)
    speck.shift(True, -20)
    sleep(down_time)
    speck.shift(True, 10)
    sleep(down_time)
    speck.shift(False, 10)
    sleep(down_time)
    speck.shift(False, -20)
    sleep(down_time)
    speck.shift(False, 10)
    sleep(down_time)

    # _____Flash Camera LED_____
    try:
        speck.Camera.LED.off()
        sleep(down_time)
        speck.Camera.LED.on()
        sleep(down_time)
    except RuntimeError:
        pass

    # _____Show Closing Jaws_____
    speck.grab()
    sleep(5)

    # _____Show Opening Jaws_____
    speck.drop()
    sleep(5)

    # _____Show Lowering Onto Crate_____
    speck.sit()
    sleep(7)
    speck.stand()
    sleep(down_time)

    # _____Show Dropping Crate_____
    speck.drop()
    sleep(down_time)

    # _____Show Sitting_____
    speck.sit()
    sleep(down_time)

