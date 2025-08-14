import mpu6050
import time
import math

# Create a new MPU6050 object
mpu = mpu6050.mpu6050(0x68)

ACCEL_OFFSET = [0, 0, 0]
GYRO_OFFSET = [0, 0, 0]


def calibrate():
    global ACCEL_OFFSET, GYRO_OFFSET
    start_time = time.time()
    total_accel = [0, 0, 0]
    total_gyro = [0, 0, 0]
    readings = 0

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

    # Calculate offsets (average)
    ACCEL_OFFSET = total_accel/readings
    GYRO_OFFSET = total_gyro/readings


# Function to read the sensor data
def read_sensor_data():
    accel = mpu.get_accel_data()
    gyro = mpu.get_gyro_data()
    temp = mpu.get_temp()
    return [accel['x'], accel['y'], accel['z']], [gyro['x'], gyro['y'], gyro['z']], temp


# Function to calculate tilt angles from accelerometer data
def calculate_angles(accel):
    print(accel)
    ax = accel[0]
    ay = accel[1]
    az = accel[2]

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

    print(f"Accelerometer data: {accel}")
    print(f"Gyroscope data: {gyro}")
    print(f"Temperature: {temp:.2f} C")
    print(f"Angles -> Roll: {angle_x:.2f}°, Pitch: {angle_y:.2f}°")
    print("-" * 40)

    time.sleep(1)
