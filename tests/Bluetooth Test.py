#!/usr/bin/env python3
import subprocess
import re
import logging
from bluezero import peripheral

# ─── CONFIGURE LOGGING ────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.DEBUG,
    format='[%(asctime)s] %(levelname)-8s %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
log = logging.getLogger(__name__)

# If you want full HCI debugging, open a separate terminal and run:
#   sudo btmon > btmon.log
# Then repro your test and inspect btmon.log for ATT_Write requests/errors.

# ─── 1) Helper: Get hci0 MAC address ─────────────────────────────────────────
def get_bt_mac():
    try:
        result = subprocess.run(
            ['hciconfig', 'hci0'],
            capture_output=True,
            text=True,
            check=True
        )
        match = re.search(r'BD Address:\s*([0-9A-F:]{17})', result.stdout)
        if not match:
            raise RuntimeError("No BD Address in hciconfig output")
        mac = match.group(1)
        log.info(f"Found adapter MAC: {mac}")
        return mac
    except Exception as e:
        log.error(f"Failed to get hci0 MAC: {e}")
        raise

# ─── 2) Bring up Bluetooth ────────────────────────────────────────────────────
def enable_bluetooth():
    for cmd in (['systemctl', 'start', 'bluetooth'],
                ['hciconfig', 'hci0', 'up']):
        try:
            subprocess.run(['sudo'] + cmd, check=True)
            log.info(f"Ran: sudo {' '.join(cmd)}")
        except subprocess.CalledProcessError as e:
            log.error(f"Command failed: sudo {' '.join(cmd)} → {e}")
            raise

# ─── 3) BLE PARAMETERS ───────────────────────────────────────────────────────
ADAPTER_ADDR = get_bt_mac()
LOCAL_NAME   = 'SPECK'
SERVICE_UUID = 'd8ed4126-c03b-499e-bf06-b69951b1fa6f'
CMD_UUID     = '529d8996-17d1-4e7c-94e9-4e84a24cbc9f'
NOTI_UUID    = 'a1b2c3d4-5678-90ab-cdef-1234567890ab'

# ─── 4) State & Robot Stub ───────────────────────────────────────────────────
_last_cmd = b''

def robot_execute_command(cmd: str) -> bool:
    """
    Replace this stub with your actual robot-control logic.
    Return True on success, False on recoverable failure.
    Raise on fatal errors.
    """
    log.info(f"[Robot] Executing command: {cmd!r}")
    # TODO: implement GPIO/serial/whatever here
    return True

# ─── 5) GATT CALLBACKS ───────────────────────────────────────────────────────
def on_connect(device):
    log.info(f"[BLE] Device connected: {device}")

def on_disconnect(device):
    log.info(f"[BLE] Device disconnected: {device}")

def on_write_cmd(value):
    global _last_cmd
    try:
        _last_cmd = bytes(value)
        cmd_str = _last_cmd.decode('utf-8', errors='replace').strip()
        log.info(f"[CMD WRITE] Received {len(_last_cmd)} bytes → {cmd_str!r}")

        success = robot_execute_command(cmd_str)
        if not success:
            raise RuntimeError("robot_execute_command returned False")

        # Acknowledge via notification
        send_notification(f"ACK: {cmd_str}")

    except Exception as e:
        log.error(f"Error in on_write_cmd: {e}")
        send_notification(f"ERR: {e}")

def on_read_cmd():
    log.debug(f"[CMD READ] Returning last command: {_last_cmd!r}")
    return list(_last_cmd)

def on_read_noti():
    # Not used; notifications are pushed programmatically
    return []

# ─── 6) NOTIFICATION HELPER ─────────────────────────────────────────────────
def send_notification(msg: str):
    data = list(msg.encode('utf-8'))
    ble_periph.update_characteristic_value(
        srv_id=1, chr_id=2, value=data
    )
    ble_periph.notify(
        srv_id=1, chr_id=2
    )
    log.info(f"[NOTIFY] {msg}")

# ─── 7) MAIN SETUP ───────────────────────────────────────────────────────────
if __name__ == '__main__':
    enable_bluetooth()

    ble_periph = peripheral.Peripheral(ADAPTER_ADDR, LOCAL_NAME)
    ble_periph.on_connect    = on_connect
    ble_periph.on_disconnect = on_disconnect

    # Primary service
    ble_periph.add_service(
        srv_id=1,
        uuid=SERVICE_UUID,
        primary=True
    )

    # Command Characteristic (iPhone → Pi)
    ble_periph.add_characteristic(
        srv_id=1, chr_id=1, uuid=CMD_UUID,
        value=[],
        notifying=False,
        flags=[
            'write-without-response',  # must be first for iOS
            'write',                   # backup write-with-response
            'read'
        ],
        write_callback=on_write_cmd,
        read_callback=on_read_cmd
    )

    # Notification Characteristic (Pi → iPhone)
    ble_periph.add_characteristic(
        srv_id=1, chr_id=2, uuid=NOTI_UUID,
        value=[],
        notifying=False,
        flags=[
            'read',
            'notify'  # adds CCCD
        ],
        read_callback=on_read_noti
    )

    log.info(f'Advertising "{LOCAL_NAME}" on {ADAPTER_ADDR}…')
    ble_periph.publish()
    ble_periph.run()
