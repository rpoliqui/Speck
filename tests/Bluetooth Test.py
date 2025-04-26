#!/usr/bin/env python3
import subprocess
import re
from bluezero import peripheral

# 1) Helper to grab hci0’s MAC dynamically
def get_bt_mac():
    res = subprocess.run(['hciconfig', 'hci0'],
                         capture_output=True, text=True, check=True)
    m = re.search(r'BD Address: ([0-9A-F:]{17})', res.stdout)
    if not m:
        raise RuntimeError('Could not find hci0 address; is Bluetooth up?')
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

# State
_last_cmd = b''

# 4) Stub for your robot logic
def robot_execute_command(cmd: str) -> bool:
    print(f"[Robot] → {cmd}")
    # TODO: send to your hardware; return False or raise on error
    return True

# 5) Callbacks
def on_connect(dev):
    print(f"[BLE] Connected: {dev}")

def on_disconnect(dev):
    print(f"[BLE] Disconnected: {dev}")

def on_write_cmd(value):
    global _last_cmd
    _last_cmd = bytes(value)
    cmd_str = _last_cmd.decode('utf-8', errors='replace').strip()
    print(f"[CMD WRITE] '{cmd_str}'")

    try:
        ok = robot_execute_command(cmd_str)
        if not ok:
            raise RuntimeError('robot_execute_command returned False')
    except Exception as e:
        send_notification(f"ERROR: {e}")
        return

    # Acknowledge back on the notify characteristic
    send_notification(f"ACK: {cmd_str}")

def on_read_cmd():
    # Let a client read back the last command if desired
    print(f"[CMD READ] returning {_last_cmd!r}")
    return list(_last_cmd)

def on_read_noti():
    # This won't be used; notifications are pushed.
    return []

# 6) Notification helper (second characteristic)
def send_notification(msg: str):
    data = list(msg.encode('utf-8'))
    peripheral.update_characteristic_value(
        srv_id=1,
        chr_id=2,
        value=data
    )
    peripheral.notify(srv_id=1, chr_id=2)
    print(f"[NOTIFY] {msg}")

# 7) Build the peripheral
peripheral = peripheral.Peripheral(ADAPTER_ADDR, LOCAL_NAME)
peripheral.on_connect    = on_connect
peripheral.on_disconnect = on_disconnect

# -- Service --
peripheral.add_service(srv_id=1, uuid=SERVICE_UUID, primary=True)

# -- Characteristic 1: Command (iPhone→Pi) --
peripheral.add_characteristic(
    srv_id=1,
    chr_id=1,
    uuid=CMD_UUID,
    value=[],
    notifying=False,
    flags=[
        'write',                   # write w/ response
        'write-without-response',  # write w/o response
        'read'                     # optional: let them read back last cmd
    ],
    write_callback=on_write_cmd,
    read_callback=on_read_cmd
)

# -- Characteristic 2: Notifications (Pi→iPhone) --
peripheral.add_characteristic(
    srv_id=1,
    chr_id=2,
    uuid=NOTI_UUID,
    value=[],
    notifying=False,
    flags=[
        'read',   # so client can discover initial state
        'notify'  # adds the CCCD for subscribe
    ],
    read_callback=on_read_noti
)

# 8) Advertise & loop
print(f'Advertising "{LOCAL_NAME}" @ {ADAPTER_ADDR}…')
peripheral.publish()
peripheral.run()
