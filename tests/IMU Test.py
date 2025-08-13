import mpu6050
import time
import math

# Create a new MPU6050 object
mpu = mpu6050.mpu6050(0x68)

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

    angle_x = math.degrees(math.atan2(ay, math.sqrt(ax**2 + az**2)))
    angle_y = math.degrees(math.atan2(-ax, math.sqrt(ay**2 + az**2)))
    angle_z = math.degrees(math.atan2(math.sqrt(ax**2 + ay**2), az))

    return angle_x, angle_y, angle_z

# Main loop
while True:
    accel, gyro, temp = read_sensor_data()
    angle_x, angle_y, angle_z = calculate_angles(accel)

    print(f"Accelerometer data: {accel}")
    print(f"Gyroscope data: {gyro}")
    print(f"Temperature: {temp:.2f} C")
    print(f"Angles -> X: {angle_x:.2f}°, Y: {angle_y:.2f}°, Z: {angle_z:.2f}°")
    print("-" * 40)

    time.sleep(1)
