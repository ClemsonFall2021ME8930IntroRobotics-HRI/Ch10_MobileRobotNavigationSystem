# Ch8_MobileRobotNavigationSystem


This repository contains the code to demonstrate content from chapter 8 Mobile Robot Navigation. In particular we show an example of the vector filed histogram technique described in the vector filed histogram section of the chapter. We employ Coppeliasim to simulate the robotics and collect lidar data. The lidar data is passed to a python program through the remote api provided by Coppeliasim. The lidar data is transformed into polar coordinates such that the objects around the robot can be classified by an angle theta and a radius from the center point of the robot r. The polar histogram is displayed throughout the program and it depicts the obstacle density around the robot. We break the 360 degrees around the robot into 18 20 degree sectors. Each sector has its own density and the decision to turn or continue straight is decided when the obstacle density surpasses a set threshold. The equation constants can be tuned in order to make the density value more sensitive to how far the robot is from an obstacle. The threshold can also be tuned to allow the robot to come closer to obstacles before deciding to take evasive action.

Requirements In order to run the code in this repository the user will need to install the following dependencies
-matplotlib library
-numpy library
-coppeliasim
-coppeliasim remote python api

Installation Instructions
matplotlib - https://www.tutorialspoint.com/how-to-install-matplotlib-in-python
numpy - https://numpy.org/install/
coppeliasim - https://coppeliarobotics.com/downloads
coppeliasim remote python api - included in installation
