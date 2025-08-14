import mpu6050
import time
import math
import numpy as np

# Create a new MPU6050 object
mpu = mpu6050.mpu6050(0x68)

ACCEL_OFFSET = np.zeros(3)  # [x, y, z]
GYRO_OFFSET = np.zeros(3)  # [x, y, z]


def calibrate():
    global ACCEL_OFFSET, GYRO_OFFSET
    start_time = time.time()
    total_accel = np.zeros(3)  # [x, y, z]
    total_gyro = np.zeros(3)  # [x, y, z]
    readings = 0
    print("Starting Calibration")
    # Collect data for calibration
    while time.time() - start_time <= 10:  # 10 seconds instead of 1000!
        accel, gyro, temp = read_sensor_data()

        total_accel[0] = total_accel[0] + accel[0]
        total_accel[1] = total_accel[1] + accel[1]
        total_accel[2] = total_accel[2] + accel[2]

        total_gyro[0] = total_gyro[0] + gyro[0]
        total_gyro[1] = total_gyro[1] + gyro[1]
        total_gyro[2] = total_gyro[2] + gyro[2]

        readings += 1

    print(f"Calibration Readings: {readings}")
    # Calculate offsets (average)
    ACCEL_OFFSET = total_accel/readings
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
print(f"Accelerometer offset: {np.array2string(ACCEL_OFFSET, precision=4)}")
print(f"Gyroscope offset: {np.array2string(GYRO_OFFSET, precision=4)}")

last_time = time.time()
sensitivity = 0.95
roll, pitch = 0.0, 0.0

# Main loop
while True:
    accel, gyro, temp = read_sensor_data()
    accel_roll, accel_pitch = accel_angles(accel)

    accel_roll -= ACCEL_OFFSET[0]
    accel_pitch -= ACCEL_OFFSET[1]

    roll_rate = gyro[0] - GYRO_OFFSET[0]
    pitch_rate = gyro[1] - GYRO_OFFSET[1]

    current_time = time.time()
    dt = current_time - last_time
    last_time = current_time

    roll = sensitivity * (roll + (dt * roll_rate)) + ((1-sensitivity) * accel_roll)
    pitch = sensitivity * (pitch + (dt * pitch_rate)) + ((1-sensitivity) * accel_pitch)

    print(f"Angles -> Roll: {roll:.2f}°, Pitch: {pitch:.2f}°")
    print("-" * 40)

    time.sleep(0.1)
