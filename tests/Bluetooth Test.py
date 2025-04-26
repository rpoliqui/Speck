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
_last_cmd = b''


def on_write(value):
    global _last_cmd
    _last_cmd = bytes(value)
    print(f"[on_write] Got {_last_cmd!r}")
    # Optionally acknowledge via notify:
    my_peripheral.notify(srv_id=1, chr_id=1)


def on_read():
    return list(_last_cmd)


# Create Peripheral instance with positional args
my_peripheral = peripheral.Peripheral(ADAPTER_ADDR, LOCAL_NAME)

# Service and characteristic setup
my_peripheral.add_service(srv_id=1, uuid=SERVICE_UUID, primary=True)

my_peripheral.add_characteristic(
    srv_id=1, chr_id=1, uuid=CMD_CHAR_UUID,
    value=[],
    notifying=False,
    flags=[
        'read',
        'write',
        'write-without-response',
        'notify'
    ],
    write_callback=on_write,
    read_callback=on_read
)

print(f'Advertising…')
my_peripheral.publish()
my_peripheral.run()
