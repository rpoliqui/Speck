"""
Ryan Poliquin, started on 2/21/2025
This code is used to quickly update Speck's code from the GitHub repository
"""

import os
import subprocess

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
    result = subprocess.run(['git', 'describe', '--tags'], capture_output=True, text=True)
    print("___________________________________________________________________________________________")
    print("Current Speck Version: %s." % result.stdout)
    print("___________________________________________________________________________________________")
else:  # Wi-Fi is not connected, Speck cannot be updated
    print("Speck cannot be updated without a wifi connection.")
