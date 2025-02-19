import os
import subprocess

subprocess.run(['apt', 'install', 'git'])  # install git on the pi
subprocess.run(['sudo', 'apt', 'autoremove'])  # remove any unnecessary packages from the pi
target_dir = os.getcwd()+"/Speck"  # define the directory where the repository will be stored
wifi_ip = subprocess.check_output(['hostname', '-I'])  # get IP address of pi
if wifi_ip is not None:  # Wi-Fi is connected, so Speck can be updated
    # Check if the repository already exists in the target directory
    print("\n\n_____________________________________________________________________________________________________\n")
    print("Installing Speck from GitHub")
    if os.path.isdir(os.path.join(target_dir, '.git')):
        # Pull the latest changes from GitHub and merge changes
        subprocess.run(['git', 'pull', 'https://github.com/rpoliqui/Speck/', 'main', '--commit', '--ff-only'])
    else:
        # If the repository doesn't exist, clone the repository from GitHub
        subprocess.run(['git', 'clone', 'https://github.com/rpoliqui/Speck/'])
    print("\n\n_____________________________________________________________________________________________________\n")
    print("Updating Raspberry Pi and All python packages")
    subprocess.run(['sudo', 'apt', 'update'])  # Update the package list
    subprocess.run(['sudo', 'apt', 'upgrade'])  # Update the packages
    subprocess.run(['sudo', 'pip', 'install', 'pyzmq==21.0.0'])  # Update pyzmq for messages, was throwing error of outdated version
    subprocess.run(['sudo', 'pip', 'install', '--upgrade', 'cypython'])  # Update pip
    subprocess.run(['sudo', 'pip', 'install', '--upgrade', 'pip'])  # Update pip
    subprocess.run(['sudo', 'pip', 'install', '--upgrade', 'gpiozero'])  # Update gpiozero
    subprocess.run(['sudo', 'pip', 'install', '--upgrade', 'numpy'])  # Update numpy
else:  # Wi-Fi is not connected, Speck cannot be updated
    print("Speck cannot be updated without a wifi connection.")
