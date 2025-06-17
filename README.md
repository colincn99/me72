# me72

## Demo Video
https://github.com/user-attachments/assets/8df22be2-8a9c-40f8-9ea4-56c10e84696a

## Descripton
ME72 capstone design project.
This repo demonstrates the usage of a camera and line laser to detect the position of the other robot and respond. This techniqued was performed on a raspberry pi with heavy use of ROS during preliminary trials.
Unfortunately, due to issues with computation speed and laser intensity, we deemed this technique to be unsuitable for the competition, and we transitioned to an arduino with line sensors and range sensors.

More information about the competition can be found at: https://me72.caltech.edu/me72/2022

## Usage info

me72/sumo/launch/sumo.launch starts the nodes necessary to run the bot

me72/sumo/scripts contains all the scripts used to write the project

The necessary node scripts are:

wheelcontrol.py: Interface with the roboclaw, send direct commands to the wheels, and send encoder data

odometry.py: Transform veloicity and angular rate commands to wheel commands and transform encoder data to position change

localize.py: Use sensor data to evaluate the location of the bot

cameratolaserscan.py: Use the line laser and camera to create a laser scan messege


The control scripts are:

teleop.py: Use keyboard to control the bot

autonomous.py: Control the robot autonomously with sensors


The other files in this folder are dependency files, python compiled files, and test scripts
