# RDA planner ROS Examples for self-driving


## Prequisites

Please install the following packages first.

- Carla simulator, 0.9.13
- [Carla ROS bridge](https://github.com/carla-simulator/ros-bridge)

## Run examples

- start the Carla simulator

```
cd $CARLA_ROOT && ./CarlaUE4.sh
```

- run the Carla ROS bridge to configure the environment

```
roslaunch rda_ros run_carla_Town04.launch
```

- Generate the traffic

```
roslaunch rda_ros generate_traffic.launch
```

- Run the RDA planner

```
roslaunch rda_ros rda_carla.launch
```

We have also provided the shell script to run the above commands in sequence, you can run the following command to in your bash shell.







