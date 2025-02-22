# Speck - The Mini Cargo Quadruped Robot Designed with Raspberry Pi
Speck is a mini quadruped robot that has been designed to pick up and carry a specially designed crate.

## Getting Started
These instructions assume that you have basic knowledge of how to connect to a Raspberry Pi via SSH and that your Raspberry Pi is connected to the internet. If your Raspberry Pi is not connected to the internet, then you will not be able to install the necessary code from this GitHub repository

### Installing Python Packages and Scripts onto your Pi
Navigate to the directory where you would like to install Speck. This can be done using the `cd` command. Note that this process will create a sub folder called Speck in your current directory.
First, install Git on you Pi with the following command.

`sudo apt install git`

Next, clone the GitHub repository into the current directory.

`git clone https://github.com/rpoliqui/Speck/`

Once the repository has been cloned, navigate into the Speck directory using `cd Speck`. Next, you will run the `install_Speck.py`
script to install all necessary packages, pull the newest code from the GitHub repository, and
make sure your Pi is set up to run Speck. To do this, run the following command:

`python3 install_Speck.py`

- Note: this script can be run at any time to update the system. This script can also be run instead of running 
`sudo apt install git` and `git clone https://github.com/rpoliqui/Speck/` to install git and clone the repository into
your current directory

## Required Equipment
Below is a list of all the hardware required to build Speck and a reference link for purchasing these items. This hardware
can be purchased from other sources. The reference link is there to provide an initial source.

| Equipment                                                                  | Quantity | Reference Link  |
|----------------------------------------------------------------------------|:---------|:---------------:|
| Raspberry Pi 4B                                                            | 1        |                 |
| MG996R 20kg Servos                                                         | 12       |                 |
| L298N Motor Driver                                                         | 1        |                 |
| Arducam 5MP 1080P HD OV5647 Camera Module                                  | 1        |                 |
| Infrared Obstacle Avoidance Sensor Module                                  | 5        |                 |
| Breakout Board Terminal Block Shield HAT                                   | 1        |                 |
| Electric Linear Actuator DC5V 30mm stroke                                  | 2        |                 |
| 18650 Rechargeable 3.7V High Capacity 3500mAh Battery                      | 2        |                 |
| Micro Limit Switch KW12-3                                                  | 2        |                 |
| Geekworm 19MM Metal Push Button Switch Latching                            | 1        |                 |
| X728 (Max 5.1V 6A) UPS & Power Management Board                            | 1        |                 |
| M2.5 Shoulder Screws, 3mm shoulder, 4mm shoulder length, 5mm thread length | 4        |                 |
| 683ZZ Ball Bearing 3mm x 7mm x 3mm                                         | 4        |                 |
| M3 x 5 machine screw                                                       | 5        |                 |
|                                                                            |          |                 |