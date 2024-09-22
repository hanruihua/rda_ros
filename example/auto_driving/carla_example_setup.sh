#!/bin/bash

cd ~/rda_ws/src

# Clone rvo_ros repository
if [ ! -d "ros-bridge" ]; then
  git clone https://github.com/hanruihua/ros-bridge
else
  echo "rvo_ros already cloned."
fi

# Build the workspace
cd ~/rda_ws
catkin_make

echo "Setup completed successfully."


