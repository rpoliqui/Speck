from bluezero import peripheral

# Callback when iPhone writes to the Pi
def on_write(value):
    print('Received from iPhone:', value.decode('utf-8'))

# Create the peripheral object using the correct argument name
my_peripheral = peripheral.Peripheral(
    adapter_name='hci0',        # ✅ Correct parameter
    local_name='SPECK'          # Name shown on iPhone
)

# Add a custom service
my_peripheral.add_service(
    srv_id=1,
    uuid='12345678-1234-5678-1234-56789abcdef0',
    primary=True
)

# Add a writable/readable/notify characteristic to that service
my_peripheral.add_characteristic(
    srv_id=1,
    chr_id=1,
    uuid='12345678-1234-5678-1234-56789abcdef1',
    value=[],
    notifying=False,
    flags=['read', 'write', 'notify'],
    write_callback=on_write
)

print("Advertising BLE service...")
my_peripheral.publish()
