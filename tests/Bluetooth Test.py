# import bluetooth
# import subprocess
# from threading import Thread
#
# import subprocess
# import time
# import os
#
#
# def setup_bluetoothctl():
#     print("\n[Setting up Bluetooth...]")
#     bluetoothctl_commands = f"""
#                             power on
#                             agent on
#                             discoverable on
#                             pairable on
#                             scan on
#                             """
#
#     # Run bluetoothctl with input commands
#     process = subprocess.Popen(['bluetoothctl'], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
#     out, err = process.communicate(bluetoothctl_commands)
#     print(out)
#     if err:
#         print("[Bluetoothctl Error]", err)
#
#
# def bluetooth_server():
#     server_sock.listen(1)
#     client_sock, address = server_sock.accept()
#     print("Client Address:", address)
#
#     try:
#         while True:
#             recv_data = client_sock.recv(1024)
#             if not recv_data:
#                 break
#             message = recv_data.decode().strip()
#             print(f"Info Received: {message}")
#             if message == "Q":
#                 print("Closing Bluetooth Connection")
#                 break
#             elif message == "M":
#                 print("M")
#     except Exception as e:
#         print("Error:", e)
#     finally:
#         client_sock.close()
#         server_sock.close()
#
#
# if __name__ == '__main__':
#     subprocess.run(['service', 'bluetooth', 'start'])  # start bluetooth on pi
#     setup_bluetoothctl()
#     server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
#     port = 3
#     server_sock.bind(("", port))
#
#     bluetooth_thread = Thread(target=bluetooth_server, daemon=True)
#     bluetooth_thread.start()

#!/usr/bin/env python3
"""
Simple RFCOMM Bluetooth server:
  - Starts bt service
  - Puts adapter into discoverable mode
  - Waits for and handles one client connection
"""

import logging
import subprocess
from threading import Thread

import bluetooth

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

PORT = 3  # RFCOMM channel

def run_cmd(cmd_list):
    logger.debug("Running: %s", cmd_list)
    res = subprocess.run(cmd_list, capture_output=True, text=True)
    if res.returncode:
        logger.error("Command failed: %s", res.stderr)
    return res.stdout

def init_bluetooth_adapter():
    cmds = ["power on", "agent on", "discoverable on", "pairable on", "scan on"]
    proc = subprocess.Popen(['bluetoothctl'], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
    out, _ = proc.communicate("\n".join(cmds))
    logger.info("Adapter init:\n%s", out)

def bluetooth_server():
    with bluetooth.BluetoothSocket(bluetooth.RFCOMM) as server_sock:
        server_sock.bind(("", PORT))
        server_sock.listen(1)
        logger.info("Listening on RFCOMM port %d", PORT)
        client_sock, addr = server_sock.accept()
        logger.info("Accepted connection from %s", addr)
        with client_sock:
            while True:
                data = client_sock.recv(1024)
                if not data or data == b"Q":
                    break
                logger.info("Recv: %s", data)
    logger.info("Server shut down")

def main():
    run_cmd(['sudo', 'service', 'bluetooth', 'start'])
    init_bluetooth_adapter()
    server_thread = Thread(target=bluetooth_server, daemon=True)
    server_thread.start()
    server_thread.join()

if __name__ == "__main__":
    main()

