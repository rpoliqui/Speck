from bluezero import peripheral

# 1) Change this to your Pi's BLE MAC (from `hciconfig` or `bluetoothctl show`)
ADAPTER_ADDR = 'D8:3A:DD:5F:A2:60'
LOCAL_NAME = 'SPECK'


# 2) Callback for writes from the iPhone
def on_write(value):
    print('Received from iPhone:', bytes(value).decode('utf-8'))


# 3) Create the Peripheral with the correct argument
my_peripheral = peripheral.Peripheral(
    adapter_address=ADAPTER_ADDR,
    local_name=LOCAL_NAME
)

# 4) Define a GATT service (UUIDs are just examples; generate your own with uuidgen or Python)
SERVICE_UUID = '12345678-1234-5678-1234-56789abcdef0'
COMMAND_CHAR_UUID = '12345678-1234-5678-1234-56789abcdef1'

my_peripheral.add_service(
    srv_id=1,
    uuid=SERVICE_UUID,
    primary=True
)

my_peripheral.add_characteristic(
    srv_id=1,
    chr_id=1,
    uuid=COMMAND_CHAR_UUID,
    value=[],
    notifying=False,
    flags=['read', 'write', 'notify'],
    write_callback=on_write
)

# 5) Start advertising and enter the event loop
print(f'▶️ Advertising "{LOCAL_NAME}" on adapter {ADAPTER_ADDR}…')
my_peripheral.publish()
