#!/bin/bash

cd ~/rda_ws/src

# Clone rvo_ros repository
if [ ! -d "rvo_ros" ]; then
  git clone https://github.com/hanruihua/rvo_ros
else
  echo "rvo_ros already cloned."
fi

if [ -n "$($SHELL -c 'echo $ZSH_VERSION')" ]; then
  echo "export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/rda_ws/devel/lib " >> ~/.zshrc
elif [ -n "$($SHELL -c 'echo $BASH_VERSION')" ]; then
  echo "export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/rda_ws/devel/lib " >> ~/.bashrc
fi

# Clone limo_ros repository
if [ ! -d "limo_ros" ]; then
  git clone https://github.com/hanruihua/limo_ros
else
  echo "limo_ros already cloned."
fi

# Build the workspace
cd ~/rda_ws
catkin_make


echo "Setup completed successfully."


