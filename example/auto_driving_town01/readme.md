# RDA planner ROS Examples for self-driving in CARLA Town01


## Prequisites

Please install the following packages first.

- [Carla 0.9.13](https://github.com/carla-simulator/carla/releases) simulator and setup the $CARLA_ROOT in your .basrhrc or .zshrc

- [Carla ROS bridge](https://github.com/carla-simulator/ros-bridge), Please set PYTHONPATH depend on your Carla ROS bridge document.

You can install the aforementioned packages by following the instructions provided in the readme.md file located within the "auto_driving" directory.

## Run examples

We provide the shell script to run the example directly by running:

```
sh run_carla_town01_example.sh
```

or 

You can also run this example by the following command step-by-step:

### Step 1. start the Carla simulator

```
cd $CARLA_ROOT && ./CarlaUE4.sh
```

### Step 2. run the Carla ROS bridge to configure the environment

```
roslaunch rda_ros run_carla_Town01.launch
```

### Step 3. Generate the traffic vehicles

```
roslaunch rda_ros generate_traffic_Town01.launch
```

### Step 4. Run rda_ros control

```
roslaunch rda_ros rda_carla_Town01.launch
```

**Note**: You can change the reference path by clicking 2D nav goal in rviz.  







