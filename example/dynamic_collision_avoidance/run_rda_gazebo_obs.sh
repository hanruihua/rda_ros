#!/bin/bash

# Step 1: Launch the Gazebo environment
gnome-terminal -- bash -c "roslaunch rda_ros gazebo_limo_env10.launch"

# Give some time for Gazebo to start
sleep 15

# Step 2: Launch the RDA control with obstacle information from Gazebo model
gnome-terminal -- bash -c "roslaunch rda_ros rda_gazebo_limo_obs.launch"