import bluetooth
import subprocess
from threading import Thread


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

    server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    port = 3
    server_sock.bind(("", port))

    bluetooth_thread = Thread(target=bluetooth_server, daemon=False)
    bluetooth_thread.start()
