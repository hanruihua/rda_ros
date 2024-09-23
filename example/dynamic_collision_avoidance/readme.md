# RDA planner ROS Examples for dynamic environment


## Prequisites

This example rely on following packages: 

- [rvo_ros](https://github.com/hanruihua/rvo_ros)

- [limo_ros](https://github.com/hanruihua/limo_ros)

You can install these packages manually or by running the provided shell script:

```bash
sh gazebo_example_setup.sh
```

## Run examples

Run dynamic collision avoidance examples with limo model in Gazebo. 

We provide the shell script to run the examples directly by running:

```
sh run_rda_gazebo_obs.sh
```

and 

```
sh run_rda_gazebo_scan.sh
```

or 

You can also run this example by the following command step-by-step:

### Step 1. launch the gazebo environment

```
roslaunch rda_ros gazebo_limo_env10.launch
```

### Step 2. launch the rda control

- If you want to obtain the obstacles information from gazebo model, please run the following command in a new terminal.

```
roslaunch rda_ros rda_gazebo_limo_obs.launch
```

- If you want to obtain the obstacles information from laser scan, please run the following command in a new terminal. 

```
roslaunch rda_ros rda_gazebo_limo_scan.launch
```

**Note:** you can change the goal by clicking rviz 2D nav goal.




