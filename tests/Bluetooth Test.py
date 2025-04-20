from bluezero import peripheral

# Callback when iPhone writes a command to the Pi
def on_write(value):
    print("Received from iPhone:", value.decode('utf-8'))


# Characteristic object with a write handler
my_characteristic = peripheral.Characteristic(
    uuid='12345678-1234-5678-1234-56789abcdef1',
    properties=['read', 'write', 'notify'],
    secure=['read', 'write'],
    value=[],  # Initial value is empty
    descriptors=None,
    read_callback=None,
    write_callback=on_write
)

# GATT Service with the characteristic
my_service = peripheral.Service(
    uuid='12345678-1234-5678-1234-56789abcdef0',
    primary=True,
    characteristics=[my_characteristic]
)

# Peripheral instance using adapter 'hci0' (default)
my_peripheral = peripheral.Peripheral(adapter_name='hci0',
                                      local_name='SPECK',
                                      services=[my_service])

print("Starting BLE GATT server...")
my_peripheral.run()
