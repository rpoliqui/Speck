import os
import subprocess
from git import Repo

target_dir = os.getcwd()+"/Speck"
wifi_ip = subprocess.check_output(['hostname', '-I'])
if wifi_ip is not None:  # Wi-Fi is connected, so Speck can be updated
    # Check if the repository already exists in the current directory
    if os.path.isdir(os.path.join(target_dir, '.git')):
        # Pull the latest changes from GitHub
        subprocess.run(['git', 'pull', 'https://github.com/rpoliqui/Speck/', 'main'])
    else:
        # If the repository doesn't exist, clone it
        Repo.clone_from('https://github.com/rpoliqui/Speck/', target_dir)

    subprocess.run(['pip', 'install', '-e', target_dir])  # Install the Speck package
    successful = True  # Speck was successfully updated
    subprocess.run(['sudo', 'apt', 'update'])  # Update the package list
    subprocess.run(['sudo', 'apt', 'upgrade'])  # Update the packages
    subprocess.run(['pip', 'install', '--upgrade', 'pip'])  # Update pip
    subprocess.run(['pip', 'install', '--upgrade', 'gpiozero'])  # Update gpiozero
    subprocess.run(['pip', 'install', '--upgrade', 'numpy'])  # Update numpy
    subprocess.run(['pip', 'install', '--upgrade', 'GitPython'])  # Update GitPython
else:  # Wi-Fi is not connected, Speck cannot be updated
    print("Speck cannot be updated without a wifi connection.")
