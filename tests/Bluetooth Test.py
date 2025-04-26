#!/usr/bin/env python3
import subprocess
import re
from bluezero import peripheral

# 1) Get the Pi’s hci0 address
def get_bt_mac():
    out = subprocess.run(
        ['hciconfig', 'hci0'],
        capture_output=True, text=True, check=True
    ).stdout
    m = re.search(r'BD Address: ([0-9A-F:]{17})', out)
    if not m:
        raise RuntimeError("hci0 not up or no MAC found")
    return m.group(1)

# 2) Bring up Bluetooth
subprocess.run(['sudo', 'systemctl', 'start', 'bluetooth'], check=True)
subprocess.run(['sudo', 'hciconfig', 'hci0', 'up'], check=True)

# 3) BLE parameters
ADAPTER = get_bt_mac()
NAME    = 'SPECK'
SVC     = 'd8ed4126-c03b-499e-bf06-b69951b1fa6f'
CMD_CHR = '529d8996-17d1-4e7c-94e9-4e84a24cbc9f'
NOT_CHR = 'a1b2c3d4-5678-90ab-cdef-1234567890ab'

_last = b''

# 4) Robot stub
def robot_execute_command(cmd):
    print(f"[ROBOT] would run: {cmd!r}")
    return True

# 5) Callbacks
def on_connect(dev):
    print(f"[BLE] Connected: {dev}")

def on_disconnect(dev):
    print(f"[BLE] Disconnected: {dev}")

def on_write_cmd(value):
    global _last
    _last = bytes(value)
    s = _last.decode('utf-8', errors='replace').strip()
    print(f"[WRITE] got {len(_last)} bytes → {s!r}")
    try:
        if not robot_execute_command(s):
            raise RuntimeError("robot reported failure")
        send_notification(f"ACK:{s}")
    except Exception as e:
        send_notification(f"ERR:{e}")

def on_read_cmd():
    print(f"[READ] returning {_last!r}")
    return list(_last)

def on_read_notif():
    return []

def send_notification(msg):
    data = list(msg.encode('utf-8'))
    ble_periph.update_characteristic_value(1, 2, data)
    ble_periph.notify(1, 2)
    print(f"[NOTIFY] {msg}")

# 6) Build
ble_periph = peripheral.Peripheral(ADAPTER, NAME)
ble_periph.on_connect    = on_connect
ble_periph.on_disconnect = on_disconnect

ble_periph.add_service( srv_id=1, uuid=SVC, primary=True )

# — Command characteristic: ONLY write-without-response + read —
ble_periph.add_characteristic(
    srv_id=1, chr_id=1, uuid=CMD_CHR,
    value=[],
    notifying=False,
    flags=[
        'write-without-response',
        'read'
    ],
    write_callback=on_write_cmd,
    read_callback=on_read_cmd
)

# — Notification characteristic —
ble_periph.add_characteristic(
    srv_id=1, chr_id=2, uuid=NOT_CHR,
    value=[],
    notifying=False,
    flags=['read', 'notify'],
    read_callback=on_read_notif
)

# 7) Advertise & loop
print(f'Advertising "{NAME}" @ {ADAPTER} …')
ble_periph.publish()
ble_periph.run()
