#!/usr/bin/env python3
import subprocess
import re
from bluezero import peripheral

# 1) Fetch hci0 MAC
def get_bt_mac():
    res = subprocess.run(['hciconfig', 'hci0'],
                         capture_output=True, text=True, check=True)
    m = re.search(r'BD Address: ([0-9A-F:]{17})', res.stdout)
    if not m:
        raise RuntimeError('Cannot find hci0 address; is Bluetooth up?')
    return m.group(1)

# 2) Bring up Bluetooth
subprocess.run(['sudo', 'systemctl', 'start', 'bluetooth'], check=True)
subprocess.run(['sudo', 'hciconfig', 'hci0', 'up'], check=True)

# 3) BLE params
ADAPTER_ADDR = get_bt_mac()
LOCAL_NAME   = 'SPECK'
SERVICE_UUID = 'd8ed4126-c03b-499e-bf06-b69951b1fa6f'
CMD_UUID     = '529d8996-17d1-4e7c-94e9-4e84a24cbc9f'
NOTI_UUID    = 'a1b2c3d4-5678-90ab-cdef-1234567890ab'

_last_cmd = b''

# 4) Your robot stub
def robot_execute_command(cmd: str) -> bool:
    print(f"[Robot] → {cmd}")
    # TODO: GPIO / serial / etc.
    return True

# 5) Callbacks
def on_connect(device):
    print(f"[BLE] Connected: {device}")

def on_disconnect(device):
    print(f"[BLE] Disconnected: {device}")

def on_write_cmd(value):
    global _last_cmd
    _last_cmd = bytes(value)
    cmd_str = _last_cmd.decode('utf-8', errors='replace').strip()
    print(f"[CMD WRITE] '{cmd_str}'")
    try:
        success = robot_execute_command(cmd_str)
        if not success:
            raise RuntimeError('robot_execute_command returned False')
        # ACK back:
        send_notification(f"ACK: {cmd_str}")
    except Exception as e:
        send_notification(f"ERROR: {e}")

def on_read_cmd():
    print(f"[CMD READ] {_last_cmd!r}")
    return list(_last_cmd)

def on_read_noti():
    # not used; notifications are pushed
    return []

# 6) Notification helper
def send_notification(msg: str):
    data = list(msg.encode('utf-8'))
    ble_periph.update_characteristic_value(srv_id=1, chr_id=2, value=data)
    ble_periph.notify(srv_id=1, chr_id=2)
    print(f"[NOTIFY] {msg}")

# 7) Build the Peripheral
ble_periph = peripheral.Peripheral(ADAPTER_ADDR, LOCAL_NAME)
ble_periph.on_connect    = on_connect
ble_periph.on_disconnect = on_disconnect

ble_periph.add_service(srv_id=1, uuid=SERVICE_UUID, primary=True)

# — Command Characteristic (iPhone→Pi) —
ble_periph.add_characteristic(
    srv_id=1, chr_id=1, uuid=CMD_UUID,
    value=[],
    notifying=False,
    flags=[
        'write-without-response',  # priority for iOS
        'write',                   # fallback write with response
        'read'
    ],
    write_callback=on_write_cmd,
    read_callback=on_read_cmd
)

# — Notification Characteristic (Pi→iPhone) —
ble_periph.add_characteristic(
    srv_id=1, chr_id=2, uuid=NOTI_UUID,
    value=[],
    notifying=False,
    flags=[
        'read',
        'notify'   # creates CCCD
    ],
    read_callback=on_read_noti
)

# 8) Advertise & loop
print(f'Advertising "{LOCAL_NAME}" @ {ADAPTER_ADDR}…')
ble_periph.publish()
ble_periph.run()
