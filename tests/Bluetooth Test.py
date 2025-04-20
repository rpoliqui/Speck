from bluezero import peripheral

# Define callback function for characteristic write
def on_write(value):
    print("Received from iPhone:", value.decode('utf-8'))

# Define the BLE GATT structure
my_peripheral = peripheral.Peripheral(adapter_addr='hci0', local_name='SPECK')

my_peripheral.add_service(srv_id=1, uuid='12345678-1234-5678-1234-56789abcdef0', primary=True)

my_peripheral.add_characteristic(srv_id=1,
                                  chr_id=1,
                                  uuid='12345678-1234-5678-1234-56789abcdef1',
                                  value=[],
                                  notifying=False,
                                  flags=['read', 'write', 'notify'],
                                  write_callback=on_write)

print("Starting BLE advertising...")
my_peripheral.publish()
