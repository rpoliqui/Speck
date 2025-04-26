import subprocess
import re
from bluezero import peripheral


# --- Helper: get hci0 MAC address automatically ---
def get_bt_mac():
    result = subprocess.run(
        ['hciconfig', 'hci0'],
        capture_output=True,
        text=True,
        check=True
    )
    match = re.search(r'BD Address: ([0-9A-F:]{17})', result.stdout)
    if match:
        return match.group(1)
    raise RuntimeError('Could not find Bluetooth MAC address. Is hci0 up?')


# --- Bring up Bluetooth ---
subprocess.run(['sudo', 'systemctl', 'start', 'bluetooth'], check=True)
subprocess.run(['sudo', 'hciconfig', 'hci0', 'up'], check=True)

# --- BLE setup ---
ADAPTER_ADDR = get_bt_mac()
LOCAL_NAME = 'SPECK'
SERVICE_UUID = 'd8ed4126-c03b-499e-bf06-b69951b1fa6f'
CMD_CHAR_UUID = '529d8996-17d1-4e7c-94e9-4e84a24cbc9f'
ERR_CHAR_UUID = 'a1b2c3d4-5678-90ab-cdef-1234567890ab'


# Stub: replace with your actual robot command executor
def robot_execute_command(cmd_str: str) -> bool:
    print(f"[Robot] Executing command: {cmd_str}")
    # TODO: send cmd_str to motors/serial/GPIO here
    # Return False or raise on failure
    return True


# Called when the phone writes to the Command Characteristic
def on_write(value):
    try:
        cmd = bytes(value).decode('utf-8').strip()
        print('Received from iPhone:', cmd)

        if not robot_execute_command(cmd):
            send_error(f"Command failed: '{cmd}'")
    except Exception as e:
        send_error(f"Error handling '{value}': {e}")


# Create the Peripheral (positional args, not keywords!)
my_peripheral = peripheral.Peripheral(ADAPTER_ADDR, LOCAL_NAME)

# 1) Add primary service
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
my_peripheral.add_characteristic(
    srv_id=1,
    chr_id=2,
    uuid=ERR_CHAR_UUID,
    value=[],
    notifying=False,
    flags=['read', 'notify']
)


# Helper to push error messages back to the phone
def send_error(msg: str):
    # Update the characteristic
    my_peripheral.update_characteristic_value(
        srv_id=1,
        chr_id=2,
        value=list(msg.encode('utf-8'))
    )
    # Notify any connected central
    my_peripheral.notify(srv_id=1, chr_id=2)
    print(f"[Error → iPhone] {msg}")


# Advertise and run
print(f'Advertising "{LOCAL_NAME}" on adapter {ADAPTER_ADDR}…')
my_peripheral.publish()
my_peripheral.run()
