from bluezero import peripheral


# Function to handle writes from the iPhone
def command_received(value):
    print(f"Received command: {value}")
    response = f"Echo: {value}"
    my_characteristic.value = response.encode()  # Respond by updating the characteristic


# Define your custom characteristic
my_characteristic = peripheral.Characteristic(
    uuid='2c10bbf0-1559-489e-9f1b-b0fb173ce1e7',
    value=[],
    notifying=True,
    flags=['read', 'write', 'notify'],
    write_callback=command_received
)

# Define your custom service
my_service = peripheral.Service(
    uuid='b4195b10-69fa-4a8c-97e9-68b065d12dee',
    primary=True,
    characteristics=[my_characteristic]
)

# Create the Peripheral
my_peripheral = peripheral.Peripheral(
    adapter_addr='D8:3A:DD:5F:A2:60',  # Use your Pi's BLE MAC
    local_name='SPECK',
    services=[my_service]
)

print("Starting BLE peripheral...")
my_peripheral.run()
