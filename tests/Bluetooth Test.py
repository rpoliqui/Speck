from bluezero import peripheral

# Callback for when iPhone writes to this characteristic
def on_write(value):
    print("Received from iPhone:", value.decode('utf-8'))

# Characteristic definition (part of a service)
characteristic = {
    'uuid': '12345678-1234-5678-1234-56789abcdef1',
    'value': [],
    'notifying': False,
    'flags': ['read', 'write', 'notify'],
    'write_callback': on_write
}

# GATT Service definition
service = {
    'uuid': '12345678-1234-5678-1234-56789abcdef0',
    'characteristics': [characteristic]
}

# BLE Peripheral (GATT server)
my_peripheral = peripheral.Peripheral(
    adapter_addr='XX:XX:XX:XX:XX:XX',   # Replace with your Pi’s MAC
    local_name='RaspiBLE',
    services=[service]
)

print("Advertising BLE service...")
my_peripheral.run()
