# RDA planner ROS Examples for self-driving


## Prequisites

- [Carla 0.9.13](https://github.com/carla-simulator/carla/releases) simulator and setup the $CARLA_ROOT in your .basrhrc or .zshrc

- [Carla ROS bridge](https://github.com/hanruihua/ros-bridge)


## Run examples

We provide the shell script to run the example directly by running:

```
sh run_carla_example.sh
```

or 

You can also run this example by the following command step-by-step:

### Step 1. start the Carla simulator

```
cd $CARLA_ROOT && ./CarlaUE4.sh
```

### Step 2. run the Carla ROS bridge to configure the environment

```
roslaunch rda_ros run_carla_Town04.launch
```

### Step 3. Generate the traffic vehicles

```
roslaunch rda_ros generate_traffic.launch
```

### Step 4. Run rda_ros control

```
roslaunch rda_ros rda_carla.launch
```

**Note**: You can change the reference path by clicking rviz 2D nav goal.  








