import subprocess
from Speck import Speck

if __name__ == "__main__":
    subprocess.run(['python3', '/home/speck/Speck/updateSpeck.py'])  # auto update Speck
    speck = Speck()  # create an instance of the Speck object to control
    speck.stand()
