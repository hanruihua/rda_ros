#!/bin/bash

# Ensure CARLA_ROOT is set
if [ -z "$CARLA_ROOT" ]; then
  echo "CARLA_ROOT is not set. Please set it in your .bashrc or .zshrc."
  exit 1
fi

# Step 1: Start the Carla simulator
echo "Starting Carla simulator..."
gnome-terminal -- bash -c "cd $CARLA_ROOT && ./CarlaUE4.sh"

# Give Carla some time to start up
sleep 10

# Step 2: Run the Carla ROS bridge
echo "Running Carla ROS bridge..."
gnome-terminal -- bash -c "roslaunch rda_ros Town04_spawn_car.launch"

# Give some time for the ROS bridge
sleep 6

# Step 3: Generate the traffic vehicles
echo "Generating traffic vehicles..."
gnome-terminal -- bash -c "roslaunch rda_ros Town04_generate_traffic.launch"

sleep 6

# Step 4: Run rda_ros control
echo "Running RDA ROS control..."
gnome-terminal -- bash -c "roslaunch rda_ros Town04_rda_carla.launch"