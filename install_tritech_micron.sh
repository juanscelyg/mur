#!/bin/bash
clear
echo "--------------------------------"
echo "Process to install tritech_micro"
echo "--------------------------------"
roscd
cd ../src
echo "--------------------------------"
echo "cloning . . ."
echo "--------------------------------"
git clone https://github.com/mcgill-robotics/ros-tritech-micron.git tritech_micron
echo "----------------------------------------------------"
echo "Install tritech micron package from PIP using rosdep"
echo "----------------------------------------------------"
rosdep update
rosdep install tritech_micron
echo "catkin make"
catkin_make
echo "Tritech micron has been instaled!"
