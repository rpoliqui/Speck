import subprocess
import re
from bluezero import peripheral


# --- Helper: get hci0 MAC address automatically ---
def get_bt_mac():
    result = subprocess.run(['hciconfig', 'hci0'], capture_output=True, text=True)
    match = re.search(r'BD Address: ([0-9A-F:]{17})', result.stdout)
    if match:
        return match.group(1)
    raise RuntimeError('Could not find Bluetooth MAC address. Is hci0 up?')


# --- Bring up Bluetooth ---
subprocess.run(['sudo', 'systemctl', 'start', 'bluetooth'])
subprocess.run(['sudo', 'hciconfig', 'hci0', 'up'])

# --- BLE setup ---
ADAPTER_ADDR = get_bt_mac()
LOCAL_NAME = 'SPECK'
SERVICE_UUID = 'd8ed4126-c03b-499e-bf06-b69951b1fa6f'
CMD_CHAR_UUID = '529d8996-17d1-4e7c-94e9-4e84a24cbc9f'
ERR_CHAR_UUID = 'a1b2c3d4-5678-90ab-cdef-1234567890ab'


# You'll replace this stub with your real robot-control code:
def robot_execute_command(cmd_str: str):
    """
    Send the given command string to your robot.
    Raise an exception or return False on failure.
    """
    print(f"[Robot] Executing command: {cmd_str}")
    # e.g. GPIO or serial write here...
    # If something bad happens, raise RuntimeError("…")
    return True


# Called by Bluezero when iPhone writes to the Command Characteristic
def on_write(value):
    try:
        cmd = bytes(value).decode('utf-8').strip()
        print('Received from iPhone:', cmd)

        # Send to robot:
        ok = robot_execute_command(cmd)
        if not ok:
            # if robot_execute_command returns False, push error
            send_error(f"Command '{cmd}' failed")
    except Exception as e:
        # On any exception, send the error back
        send_error(f"Error handling '{value}': {e}")


# Create the peripheral and add services/chars
my_peripheral = peripheral.Peripheral(adapter_addr=ADAPTER_ADDR,
                                      local_name=LOCAL_NAME)

# 1) Primary service
my_peripheral.add_service(
    srv_id=1,
    uuid=SERVICE_UUID,
    primary=True
)

# 2) Command Characteristic (iPhone → Pi)
my_peripheral.add_characteristic(
    srv_id=1,
    chr_id=1,
    uuid=CMD_CHAR_UUID,
    value=[],
    notifying=False,
    flags=['write'],
    write_callback=on_write
)

# 3) Error Characteristic (Pi → iPhone)
#    Readable so phone can pull latest, and Notifiable so we can push
my_peripheral.add_characteristic(
    srv_id=1,
    chr_id=2,
    uuid=ERR_CHAR_UUID,
    value=[],
    notifying=False,
    flags=['read', 'notify']
)


# Helper to update & notify error messages
def send_error(msg: str):
    """
    Writes `msg` into the Error characteristic and notifies
    any connected central (the iPhone).
    """
    # Update the characteristic's value
    my_peripheral.update_characteristic_value(
        srv_id=1,
        chr_id=2,
        value=list(msg.encode('utf-8'))
    )
    # Send out a notification
    my_peripheral.notify(
        srv_id=1,
        chr_id=2
    )
    print(f"[Error -> iPhone] {msg}")


# Start advertising & GATT server
print(f'Advertising "{LOCAL_NAME}" on {ADAPTER_ADDR}…')
my_peripheral.publish()

# Blocking loop: handles GATT requests & notifications
my_peripheral.run()
