#!/bin/bash

echo "Setting up Intelligent Robotics practice..."

# Move all Python scripts to the scripts folder
mkdir -p /home/docker/catkin_ws/src/robotica_inteligente/scripts
mv *.py /home/docker/catkin_ws/src/robotica_inteligente/scripts/

# Move launch files to the launch folder
mkdir -p /home/phg18/Intelligent_robotics_practice/src/robotica_inteligente/launch
mv *.launch /home/phg18/Intelligent_robotics_practice/src/robotica_inteligente/launch/

echo "Files have been organized successfully."
echo "Now you can run: catkin_make and source devel/setup.bash"

