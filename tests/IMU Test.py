import mpu6050
import time
import math

# Create a new MPU6050 object
mpu = mpu6050.mpu6050(0x68)

ACCEL_OFFSET = 0
GYRO_OFFSET = 0


def calibrate():
    global ACCEL_OFFSET, GYRO_OFFSET
    start_time = time.time()
    total_accel = {'x': 0, 'y': 0, 'z': 0}
    total_gyro = {'x': 0, 'y': 0, 'z': 0}
    readings = 0

    # Collect data for calibration
    while time.time() - start_time <= 10:  # 10 seconds instead of 1000!
        accel, gyro, temp = read_sensor_data()

        total_accel['x'] += accel['x']
        total_accel['y'] += accel['y']
        total_accel['z'] += accel['z']

        total_gyro['x'] += gyro['x']
        total_gyro['y'] += gyro['y']
        total_gyro['z'] += gyro['z']

        readings += 1
        time.sleep(0.01)  # 100 Hz sampling

    # Calculate offsets (average)
    ACCEL_OFFSET = {axis: total_accel[axis] / readings for axis in total_accel}
    GYRO_OFFSET = {axis: total_gyro[axis] / readings for axis in total_gyro}


# Function to read the sensor data
def read_sensor_data():
    accel = mpu.get_accel_data()
    gyro = mpu.get_gyro_data()
    temp = mpu.get_temp()
    return accel, gyro, temp


# Function to calculate tilt angles from accelerometer data
def calculate_angles(accel):
    ax = accel['x']
    ay = accel['y']
    az = accel['z']

    angle_x = math.degrees(math.atan2(ay, math.sqrt(ax ** 2 + az ** 2)))
    angle_y = math.degrees(math.atan2(-ax, math.sqrt(ay ** 2 + az ** 2)))

    return angle_x, angle_y


calibrate()
print(f"Accelerometer offset: {ACCEL_OFFSET}")
print(f"Gyroscope offset: {GYRO_OFFSET}")

# Main loop
while True:
    accel, gyro, temp = read_sensor_data()
    angle_x, angle_y = calculate_angles(accel)

    print(f"Accelerometer data: {accel - ACCEL_OFFSET}")
    print(f"Gyroscope data: {gyro - GYRO_OFFSET}")
    print(f"Temperature: {temp:.2f} C")
    print(f"Angles -> Roll: {angle_x:.2f}°, Pitch: {angle_y:.2f}°")
    print("-" * 40)

    time.sleep(1)
