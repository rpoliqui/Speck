import subprocess
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
# setup pi to enable bluetooth connection
subprocess.run(['sudo', 'systemctl', 'start', 'bluetooth'])  # start bluetooth on pi
subprocess.run(['sudo', 'hciconfig', 'hci0', 'up'])  # start bluetooth on pi
bluetoothctl_commands = f"""                                                               
                                    power on                                               
                                    manufacturer 0xffff 0x12 0x34                          
                                    name SPECK                                             
                                    register-service e2d36f99-8909-4136-9a49-d825508b297b  
                                    yes                                                    
                                    register-characteristic 0x1234 read                    
                                    07                                                     
                                    register-characteristic 0x5678 read,write              
                                    13                                                     
                                    register-application                                   
                                    advertise on                                           
                                    pairable on                                            
                                    """

# Run bluetoothctl with input commands
process = subprocess.Popen(['bluetoothctl'], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                           stderr=subprocess.PIPE, text=True)
out, err = process.communicate(bluetoothctl_commands)
if err:
    print("[Bluetoothctl Error]", err)
print(f'Advertising "{LOCAL_NAME}" on adapter {ADAPTER_ADDR}…')
my_peripheral.publish()
