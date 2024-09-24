# RDA planner ROS Examples for self-driving


## Prequisites

- [Carla 0.9.13](https://github.com/carla-simulator/carla/releases) simulator and setup the $CARLA_ROOT in your .basrhrc or .zshrc

- [Carla ROS bridge](https://github.com/carla-simulator/ros-bridge), Please set PYTHONPATH depend on your Carla ROS bridge document.


You can install the Carla ROS bridge manually or by the following command:

```
sh setup_carla_example.sh
```

Please make sure you have set the $CARLA_ROOT and PYTHONPATH successfully. Following is the example of setting the $CARLA_ROOT and PYTHONPATH in your .bashrc or .zshrc:

``` 
export CARLA_ROOT = PATH TO YOUR CARLA ROOT
export PYTHONPATH="${CARLA_ROOT}/PythonAPI/carla/":"${CARLA_ROOT}/PythonAPI/carla/dist/carla-0.9.13-py3.7-linux-x86_64.egg":${PYTHONPATH}
```

- Install python packages by the following command:

```
pip install transforms3d pygame
```

## Run examples

We provide the shell script to run the examples directly by running:

```
sh run_carla_example_Town02.sh
```

and 

```
sh run_carla_example_Town04.sh
```

or 

You can also run Town04 example by the following command step-by-step:

### Step 1. start the Carla simulator

```
cd $CARLA_ROOT && ./CarlaUE4.sh
```

### Step 2. run the Carla ROS bridge to configure the environment

```
roslaunch rda_ros Town04_spawn_car.launch
```

### Step 3. Generate the traffic vehicles

```
roslaunch rda_ros Town04_generate_traffic.launch
```

### Step 4. Run rda_ros control

```
roslaunch rda_ros Town04_rda_carla.launch
```

For Town02 example, simply replace the Town04 with Town02 in the above commands.

**Note**: You can change the reference path by clicking 2D nav goal in rviz.  








