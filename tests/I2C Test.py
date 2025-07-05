from smbus2 import SMBus, i2c_msg
import time

# I2C bus number (usually 1 on Raspberry Pi)
bus_number = 1
# I2C address of your device
device_address = 0x40  # Replace with your device's address

try:
    # Create an SMBus object for the specified bus
    bus = SMBus(bus_number)

    # Example: Write a byte to a register
    register_address = 0x00  # Replace with the register address you want to write to
    data_to_write = 0x55  # Replace with the data you want to write
    bus.write_byte_data(device_address, register_address, data_to_write)
    print(f"Successfully wrote {data_to_write} to register {register_address} at address {device_address}")

    # Example: Read a byte from a register
    read_register_address = 0x01  # Replace with the register address you want to read from
    received_data = bus.read_byte_data(device_address, read_register_address)
    print(f"Read {received_data} from register {read_register_address} at address {device_address}")

    # Example: Read multiple bytes from a register
    read_register_address = 0x02
    number_of_bytes = 2
    read_data = bus.read_i2c_block_data(device_address, read_register_address, number_of_bytes)
    print(f"Read {read_data} from register {read_register_address} at address {device_address}")


except Exception as e:
    print(f"An error occurred: {e}")

finally:
    # Close the I2C bus connection
    if 'bus' in locals():
        bus.close()