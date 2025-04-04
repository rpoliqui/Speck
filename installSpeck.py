"""
Ryan Poliquin, started on 2/12/2025
This code is used to set up a Raspberry Pi to run Speck. It will install all necessary packages and properly configure
the Pi for ease of use.
"""

import os
import subprocess

subprocess.run(['sudo', 'apt', 'install', '-y', 'git'])  # install git on the pi
target_dir = os.getcwd()  # define the directory where the repository will be stored, assume this will be the current directory
if os.path.isdir(os.path.join(target_dir, 'Speck')):  # if the directory already has a Speck sub folder
    target_dir = target_dir + "/Speck"  # set this sub folder as the target directory
wifi_ip = subprocess.check_output(['hostname', '-I'])  # get IP address of pi
if wifi_ip is not None:  # Wi-Fi is connected, so Speck can be updated
    # Check if the repository already exists in the target directory
    print("\n\n_____________________________________________________________________________________________________")
    print("Installing Speck from GitHub\n")
    if os.path.isdir(os.path.join(target_dir, '.git')):
        # Pull the latest changes from GitHub and merge changes
        subprocess.run(['git', '-C', target_dir, 'pull', 'origin', 'main', '--ff-only'])
    else:
        # If the repository doesn't exist, clone the repository from GitHub
        subprocess.run(['git', 'clone', 'https://github.com/rpoliqui/Speck/', target_dir])
    print("\n\n_____________________________________________________________________________________________________")
    print("Updating Raspberry Pi and All python packages\n")
    subprocess.run(['sudo', 'apt', '-y', 'update'])  # Update the package list
    subprocess.run(['sudo', 'apt', '-y', 'upgrade'])  # Update the packages
    subprocess.run(['sudo', 'apt', '-y', 'autoremove'])  # Remove any unnecessary packages from the pi
    subprocess.run(['sudo', 'apt', 'install', 'pigpio'])  # Install pigpio for improved pin control
    subprocess.run(['sudo', 'systemctl', 'enable', 'pigpiod'])  # Enable the daemon to run at time of boot
    subprocess.run(['sudo', 'systemctl', 'start', 'pigpiod'])  # Start the daemon now to prevent rebooting
    subprocess.run(['sudo', 'apt-get', 'install', 'python3-opencv'])  # Install opencv package for image processing
    subprocess.run(['sudo', 'apt-get', 'install', 'sklearn'])  # Install sklearn package for image processing
    subprocess.run(['pip', 'install', 'pyzmq==21.0.0'])  # Update pyzmq for messages, was throwing error of outdated version
    subprocess.run(['pip', 'install', '--upgrade', 'pip'])  # Update pip
    subprocess.run(['pip', 'install', '--upgrade', 'gpiozero'])  # Update gpiozero
    subprocess.run(['pip', 'install', '--upgrade', 'picamzero'])  # Update pi camera package
    subprocess.run(['pip', 'install', '--upgrade', 'numpy'])  # Update numpy
else:  # Wi-Fi is not connected, Speck cannot be updated
    print("Speck cannot be updated without a wifi connection.")
