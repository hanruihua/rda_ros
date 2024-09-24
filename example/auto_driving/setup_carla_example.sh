#!/bin/bash

cd ~/rda_ws/src

# Clone rvo_ros repository
if [ ! -d "ros-bridge" ]; then
  git clone https://github.com/carla-simulator/ros-bridge
else
  echo "ros bridge already cloned."
fi

# Build the workspace
cd ~/rda_ws
catkin_make

echo "Build Carla ROS bridge completed successfully."


