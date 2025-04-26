#!/usr/bin/env python3
import subprocess
import re
import logging
from bluezero import peripheral

# Set up logging for debugging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

# Function to get the Bluetooth MAC address
def get_bt_mac():
    try:
        output = subprocess.check_output("hciconfig", stderr=subprocess.STDOUT)
        match = re.search(r"([0-9A-F]{2}[:]){5}[0-9A-F]{2}", output.decode())
        if match:
            return match.group(0)
    except subprocess.CalledProcessError as e:
        logging.error("Error getting Bluetooth MAC address: %s", e)
    return None

# Function to enable Bluetooth if not already enabled
def enable_bluetooth():
    try:
        subprocess.check_call("sudo systemctl start bluetooth", shell=True)
        subprocess.check_call("sudo hciconfig hci0 up", shell=True)
        logging.info("Bluetooth enabled successfully.")
    except subprocess.CalledProcessError as e:
        logging.error("Failed to enable Bluetooth: %s", e)

# BLE parameters
ADAPTER_ADDR = get_bt_mac()
LOCAL_NAME = 'SPECK'
SERVICE_UUID = 'd8ed4126-c03b-499e-bf06-b69951b1fa6f'
CMD_UUID = '529d8996-17d1-4e7c-94e9-4e84a24cbc9f'
NOTI_UUID = 'a1b2c3d4-5678-90ab-cdef-1234567890ab'

_last_cmd = b''

# Callback for executing the received command
def robot_execute_command(cmd):
    logging.info(f"[Robot] Received command: {cmd!r}")
    return True

# Callback for when a device connects
def on_connect(dev):
    logging.info(f"[BLE] Device connected: {dev}")

# Callback for when a device disconnects
def on_disconnect(dev):
    logging.info(f"[BLE] Device disconnected: {dev}")

# Callback for writing data to the command characteristic
def on_write_cmd(value):
    global _last_cmd
    _last_cmd = bytes(value)
    cmd = _last_cmd.decode('utf-8', errors='replace').strip()
    logging.info(f"[CMD WRITE] Command: '{cmd}'")
    try:
        if not robot_execute_command(cmd):
            raise RuntimeError("Command execution failed.")
        send_notification(f"ACK:{cmd}")
    except Exception as e:
        logging.error(e)
        send_notification(f"ERR:{e}")

# Callback for reading the last command
def on_read_cmd():
    logging.debug(f"[CMD READ] Returning: {_last_cmd!r}")
    return list(_last_cmd)

# Callback for reading notifications (currently empty)
def on_read_noti():
    return []

# Function to send a notification
def send_notification(msg):
    data = list(msg.encode('utf-8'))
    ble_periph.update_characteristic_value(1, 2, data)
    ble_periph.notify(1, 2)
    logging.info(f"[NOTIFY] Sent: {msg}")

# Main function to set up and run the BLE peripheral
if __name__ == '__main__':
    enable_bluetooth()

    # Create a BLE peripheral object
    ble_periph = peripheral.Peripheral(ADAPTER_ADDR, LOCAL_NAME)
    ble_periph.on_connect = on_connect
    ble_periph.on_disconnect = on_disconnect

    # Add the custom service
    ble_periph.add_service(srv_id=1, uuid=SERVICE_UUID, primary=True)

    # Command characteristic (chr_id=1)
    ble_periph.add_characteristic(
        srv_id=1, chr_id=1, uuid=CMD_UUID,
        value=[],
        notifying=False,
        flags=['write-without-response', 'write', 'read'],
        write_callback=on_write_cmd,
        read_callback=on_read_cmd
    )
    # Descriptor: User Description (0x2901)
    ble_periph.add_descriptor(
        srv_id=1, chr_id=1, dsc_id=1,
        uuid='2901',
        value=list(b'Robot Command')
    )
    # Descriptor: Presentation Format (0x2904)
    # Format: uint8, exponent 0, unit 0x2700 (unitless), namespace 1, description 0
    ble_periph.add_descriptor(
        srv_id=1, chr_id=1, dsc_id=2,
        uuid='2904',
        value=[0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    )

    # Notification characteristic (chr_id=2)
    ble_periph.add_characteristic(
        srv_id=1, chr_id=2, uuid=NOTI_UUID,
        value=[],
        notifying=False,
        flags=['read', 'notify'],
        read_callback=on_read_noti
    )
    # Descriptor: User Description (0x2901)
    ble_periph.add_descriptor(
        srv_id=1, chr_id=2, dsc_id=1,
        uuid='2901',
        value=list(b'Robot Ack/Error')
    )
    # Descriptor: Presentation Format (0x2904)
    ble_periph.add_descriptor(
        srv_id=1, chr_id=2, dsc_id=2,
        uuid='2904',
        value=[0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    )

    logging.info(f'Advertising "{LOCAL_NAME}" on {ADAPTER_ADDR}…')
    ble_periph.publish()

    # Run the BLE peripheral
    ble_periph.run()
