# RDA planner ROS Examples for dynamic environment


## Prequisites

Please install the following packages first.

- [rvo_ros](https://github.com/hanruihua/rvo_ros)

- [limo_ros](https://github.com/hanruihua/limo_ros)


## Run examples

Run dynamic collision avoidance examples with limo model in Gazebo.

First launch the gazebo environment

```
roslaunch rda_ros gazebo_limo_env20.launch
```

If you want to obtain the obstacles information from gazebo model, please run the following command in a new terminal.

```
roslaunch rda_ros rda_gazebo_limo_obs.launch
```

If you want to obtain the obstacles information from laser scan, please run the following command in a new terminal. 

```
roslaunch rda_ros rda_gazebo_limo_scan.launch
```





