#!/usr/bin/env python3
import subprocess
import re
from bluezero import peripheral

# --- 1) Helper to get the Pi's hci0 MAC address ---
def get_bt_mac():
    result = subprocess.run(
        ['hciconfig', 'hci0'],
        capture_output=True,
        text=True,
        check=True
    )
    match = re.search(r'BD Address: ([0-9A-F:]{17})', result.stdout)
    if not match:
        raise RuntimeError('Unable to find hci0 MAC address. Is Bluetooth up?')
    return match.group(1)

# --- 2) Bring up Bluetooth ---
subprocess.run(['sudo', 'systemctl', 'start', 'bluetooth'], check=True)
subprocess.run(['sudo', 'hciconfig', 'hci0', 'up'], check=True)

# --- 3) BLE parameters ---
ADAPTER_ADDR  = get_bt_mac()
LOCAL_NAME    = 'SPECK'
SERVICE_UUID  = 'd8ed4126-c03b-499e-bf06-b69951b1fa6f'
CHAR_UUID     = '529d8996-17d1-4e7c-94e9-4e84a24cbc9f'

# Keep the last written value around for reads/notifications
_last_value = b''

# --- 4) Robot command stub ---
def robot_execute_command(cmd_str: str) -> bool:
    """
    Replace this stub with your actual robot-control logic.
    Return True on success, False on recoverable failure.
    Raise on fatal errors.
    """
    print(f"[Robot] Executing: {cmd_str}")
    # TODO: hook up GPIO, serial, etc.
    return True

# --- 5) Callbacks ---
def on_connect(device_address):
    print(f"[BLE] Device connected: {device_address}")

def on_disconnect(device_address):
    print(f"[BLE] Device disconnected: {device_address}")

def on_write(value):
    global _last_value
    try:
        _last_value = bytes(value)
        cmd = _last_value.decode('utf-8', errors='replace').strip()
        print(f"[on_write] Got {len(value)} bytes: '{cmd}'")

        # Send to robot
        ok = robot_execute_command(cmd)
        if not ok:
            send_error(f"Command failed: '{cmd}'")

        # Acknowledge via notify back on the same characteristic
        my_peripheral.notify(srv_id=1, chr_id=1)

    except Exception as e:
        print(f"[on_write error] {e}")
        try:
            send_error(f"Internal error: {e}")
        except Exception as ne:
            print(f"[send_error failed] {ne}")

def on_read():
    print(f"[on_read] Returning {len(_last_value)} bytes")
    return list(_last_value)

# --- 6) Error notification helper ---
def send_error(msg: str):
    """
    Write `msg` into the characteristic and send a notify.
    """
    data = list(msg.encode('utf-8'))
    my_peripheral.update_characteristic_value(
        srv_id=1,
        chr_id=1,
        value=data
    )
    my_peripheral.notify(srv_id=1, chr_id=1)
    print(f"[Error → iPhone] {msg}")

# --- 7) Build Peripheral ---
my_peripheral = peripheral.Peripheral(ADAPTER_ADDR, LOCAL_NAME)

# Register connect/disconnect hooks
my_peripheral.on_connect    = on_connect
my_peripheral.on_disconnect = on_disconnect

# Add primary service
my_peripheral.add_service(
    srv_id=1,
    uuid=SERVICE_UUID,
    primary=True
)

# Add characteristic (read, write, notify)
my_peripheral.add_characteristic(
    srv_id=1,
    chr_id=1,
    uuid=CHAR_UUID,
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

# --- 8) Start advertising & event loop ---
print(f'Advertising "{LOCAL_NAME}" on {ADAPTER_ADDR}…')
my_peripheral.publish()
my_peripheral.run()
