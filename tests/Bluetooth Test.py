import bluetooth
import subprocess
from threading import Thread

import subprocess
import time
import os


def setup_bluetoothctl():
    print("\n[Setting up Bluetooth...]")
    bluetoothctl_commands = f"""
                            power on
                            agent on
                            discoverable on
                            pairable on
                            scan on
                            """

    # Run bluetoothctl with input commands
    process = subprocess.Popen(['bluetoothctl'], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    out, err = process.communicate(bluetoothctl_commands)
    print(out)
    if err:
        print("[Bluetoothctl Error]", err)


def bluetooth_server():
    server_sock.listen(1)
    client_sock, address = server_sock.accept()
    print("Client Address:", address)

    try:
        while True:
            recv_data = client_sock.recv(1024)
            if not recv_data:
                break
            message = recv_data.decode().strip()
            print(f"Info Received: {message}")
            if message == "Q":
                print("Closing Bluetooth Connection")
                break
            elif message == "M":
                print("M")
    except Exception as e:
        print("Error:", e)
    finally:
        client_sock.close()
        server_sock.close()


if __name__ == '__main__':
    subprocess.run(['service', 'bluetooth', 'start'])  # start bluetooth on pi
    setup_bluetoothctl()
    server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    port = 3
    server_sock.bind(("", port))

    bluetooth_thread = Thread(target=bluetooth_server, daemon=True)
    bluetooth_thread.start()
