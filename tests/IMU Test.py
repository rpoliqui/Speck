import mpu6050
import time
import math
import numpy as np

# Create a new MPU6050 object
mpu = mpu6050.mpu6050(0x68)

ANGLE_OFFSET = np.zeros(2)  # [pitch, roll]
GYRO_OFFSET = np.zeros(3)  # [x, y, z]


def calibrate():
    global ANGLE_OFFSET, GYRO_OFFSET
    start_time = time.time()
    total_gyro = np.zeros(3)  # [x, y, z]
    total_angles = np.zeros(2)  # [pitch, roll]
    readings = 0
    print("Starting Calibration")
    # Collect data for calibration
    last_time = time.time()
    sensitivity = 0.95
    roll, pitch = 0.0, 0.0
    while time.time() - start_time <= 3:  # 3 second calibration
        accel, gyro, temp = read_sensor_data()
        gyro = gyro - GYRO_OFFSET
        accel_roll, accel_pitch = accel_angles(accel)

        roll_rate = gyro[0]
        pitch_rate = gyro[1]

        # Measure time past in last cycle
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time

        # Apply Karman Filter to measure pitch and roll
        pitch = sensitivity * (pitch + (dt * pitch_rate)) + ((1 - sensitivity) * accel_pitch)
        roll = sensitivity * (roll + (dt * roll_rate)) + ((1 - sensitivity) * accel_roll)

        total_angles[0] += pitch
        total_angles[1] += roll

        total_gyro[0] = total_gyro[0] + gyro[0]
        total_gyro[1] = total_gyro[1] + gyro[1]
        total_gyro[2] = total_gyro[2] + gyro[2]

        readings += 1

    print(f"Calibration Readings: {readings}")
    # Calculate offsets (average)
    ANGLE_OFFSET = total_angles/readings
    print(f"Angle Offsets: {ANGLE_OFFSET}")
    GYRO_OFFSET = total_gyro/readings


# Function to read the sensor data
def read_sensor_data():
    accel = mpu.get_accel_data()
    gyro = mpu.get_gyro_data()
    temp = mpu.get_temp()
    return np.array([accel['x'], accel['y'], accel['z']]), np.array([gyro['x'], gyro['y'], gyro['z']]), temp


# Function to calculate tilt angles from accelerometer data
def accel_angles(accel):
    ax = accel[0]
    ay = accel[1]
    az = accel[2]

    angle_x = math.degrees(math.atan2(ay, math.sqrt(ax ** 2 + az ** 2)))
    angle_y = math.degrees(math.atan2(-ax, math.sqrt(ay ** 2 + az ** 2)))

    return angle_x, angle_y


calibrate()
last_time = time.time()
sensitivity = 0.95
roll, pitch = 0.0, 0.0

# Main loop
while True:
    accel, gyro, temp = read_sensor_data()
    gyro = gyro - GYRO_OFFSET
    accel_roll, accel_pitch = accel_angles(accel)

    roll_rate = gyro[0]
    pitch_rate = gyro[1]

    # Measure time past in last cycle
    current_time = time.time()
    dt = current_time - last_time
    last_time = current_time

    # Apply Karman Filter to measure pitch and roll
    roll = sensitivity * (roll + (dt * roll_rate)) + ((1-sensitivity) * accel_roll)
    pitch = sensitivity * (pitch + (dt * pitch_rate)) + ((1-sensitivity) * accel_pitch)

    print(f"Angles -> Roll: {roll - ANGLE_OFFSET[1]:.2f}°, Pitch: {pitch - ANGLE_OFFSET[0]:.2f}°")
    print("-" * 40)

    time.sleep(0.1)
