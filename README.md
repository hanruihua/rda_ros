# rda_ros

This is the ROS Wrapper of the [RDA planner](https://github.com/hanruihua/RDA-planner)

## Preqrequisite

Please install the following packages first.

- Python >= 3.8
- [ROS Noetic](https://wiki.ros.org/noetic)
- [RDA_planner](https://github.com/hanruihua/RDA_planner)

*Note: We recommend using Python 3.8 and Conda to manage Python environments, as some examples are based on CARLA.*

## Test Environment 
- Ubuntu 20.04

## Installation

You can install the package by running the commands below:

```
mkdir -p ~/rda_ws/src
cd ~/rda_ws/src
git clone https://github.com/hanruihua/rda_ros
cd ~/rda_ws && catkin_make
cd ~/rda_ws/src/rda_ros 
sh source_setup.sh && source ~/rda_ws/devel/setup.sh && rosdep install rda_ros 
```

## Node API

### Published Topics

| Topic Name         | Message Type                     | Description                                                  |
| ------------------ | -------------------------------- | ------------------------------------------------------------ |
| `/rda_cmd_vel`     | `geometry_msgs/Twist`            | Publishes velocity commands for the robot's movement.        |
| `/rda_opt_path`    | `nav_msgs/Path`                  | Publishes the optimized path generated by the RDA planner.   |
| `/rda_ref_path`    | `nav_msgs/Path`                  | Publishes the reference path that the robot should follow.   |
| `/rda_ref_states`  | `nav_msgs/Path`                  | Publishes the reference states trajectory for visualization. |
| `/rda_obs_markers` | `visualization_msgs/MarkerArray` | Publishes obstacle markers for visualization in RViz.        |

### Subscribed Topics

| Topic Name       | Message Type                         | Description                                                                              |
| ---------------- | ------------------------------------ | ---------------------------------------------------------------------------------------- |
| `/rda_obstacles` | `costmap_converter/ObstacleArrayMsg` | Subscribes to obstacle information when `use_scan_obstacle` is set to `False`.           |
| `/scan`          | `sensor_msgs/LaserScan`              | Subscribes to laser scan data for obstacle detection when `use_scan_obstacle` is `True`. |
| `/rda_sub_path`  | `nav_msgs/Path`                      | Subscribes to the reference path that the RDA planner should follow.                     |
| `/rda_goal`      | `geometry_msgs/PoseStamped`          | Subscribes to the goal point to generate the reference path for rda planner              |

### Parameters

#### Robot Information

| Parameter               | Type             | Default Value | Description                                                                                          |
| ----------------------- | ---------------- | ------------- | ---------------------------------------------------------------------------------------------------- |
| `~robot_info`           | `dict`           |               | Configuration dictionary for robot specifications.                                                   |
| `~robot_info.vertices`  | `list` or `None` | `None`        | Vertices defining the robot's shape. If `None`, uses length and width to create a rectangular shape. |
| `~robot_info.radius`    | `float`          | `None`        | Radius of the robot (used if the robot shape is circular).                                           |
| `~robot_info.max_speed` | `list`           | `[10, 1]`     | Maximum speed parameters `[linear, angular]` or `[linear, steer]`.                                   |
| `~robot_info.max_acce`  | `list`           | `[10, 0.5]`   | Maximum acceleration parameters `[linear, angular]` or `[linear, steer]`.                            |
| `~robot_info.length`    | `float`          | `2`           | Length of the robot (used if vertices are not provided).                                             |
| `~robot_info.width`     | `float`          | `1`           | Width of the robot (used if vertices are not provided).                                              |
| `~robot_info.wheelbase` | `float`          | `1.5`         | Wheelbase of the robot.                                                                              |
| `~robot_info.dynamics`  | `string`         | `"diff"`      | Type of robot dynamics (e.g., differential: `diff`, ackermann: `acker`).                             |
| `~robot_info.cone_type` | `string`         | `"Rpositive"` | Type of collision cone used, polygon: `Rpositive`, circle: `norm2`                                   |

#### RDA MPC Configuration

| Parameter               | Type    | Default Value | Description                                                |
| ----------------------- | ------- | ------------- | ---------------------------------------------------------- |
| `~receding`             | `int`   | `10`          | Receding horizon parameter for MPC.                        |
| `~iter_num`             | `int`   | `2`           | Number of iterations for the MPC solver.                   |
| `~enable_reverse`       | `bool`  | `False`       | Enables reverse movement if set to `True`.                 |
| `~sample_time`          | `float` | `0.1`         | Sampling time interval for the MPC.                        |
| `~process_num`          | `int`   | `4`           | Number of parallel processes for MPC computation.          |
| `~accelerated`          | `bool`  | `True`        | Enables accelerated computation in MPC.                    |
| `~time_print`           | `bool`  | `False`       | Enables time logging for MPC operations.                   |
| `~obstacle_order`       | `bool`  | `True`        | Determines if obstacle ordering by distance is applied.    |
| `~max_edge_num`         | `int`   | `5`           | Maximum number of edges to consider for obstacles.         |
| `~max_obs_num`          | `int`   | `5`           | Maximum number of obstacles to consider.                   |
| `~goal_index_threshold` | `int`   | `1`           | Threshold for goal index determination.                    |
| `~iter_threshold`       | `float` | `0.2`         | Iteration threshold for convergence in MPC.                |
| `~slack_gain`           | `float` | `8`           | Gain for slack variables in MPC constraints.               |
| `~max_sd`               | `float` | `1.0`         | Maximum safety distance.                                   |
| `~min_sd`               | `float` | `0.1`         | Minimum safety distance.                                   |
| `~ws`                   | `float` | `1.0`         | Weight for the state in the cost function.                 |
| `~wu`                   | `float` | `0.5`         | Weight for the control inputs in the cost function.        |
| `~ro1`                  | `float` | `200`         | Weight parameter for the first term in the cost function.  |
| `~ro2`                  | `float` | `1.0`         | Weight parameter for the second term in the cost function. |

#### Reference Speed

| Parameter    | Type    | Default Value | Description                    |
| ------------ | ------- | ------------- | ------------------------------ |
| `~ref_speed` | `float` | `4.0`         | Reference speed for the robot. |

#### Scan Configuration

| Parameter            | Type    | Default Value | Description                                                  |
| -------------------- | ------- | ------------- | ------------------------------------------------------------ |
| `~use_scan_obstacle` | `bool`  | `False`       | Determines whether to use laser scan for obstacle detection. |
| `~scan_eps`          | `float` | `0.2`         | Epsilon parameter for DBSCAN clustering on scan data.        |
| `~scan_min_samples`  | `int`   | `6`           | Minimum number of samples for a cluster in DBSCAN.           |

#### Reference Paths

| Parameter     | Type     | Default Value | Description                                                        |
| ------------- | -------- | ------------- | ------------------------------------------------------------------ |
| `~waypoints`  | `list`   | `[]`          | List of waypoints for generating the reference path.               |
| `~loop`       | `bool`   | `False`       | Determines if the path should loop upon reaching the end.          |
| `~curve_type` | `string` | `"dubins"`    | Type of curve generator used (e.g., `dubins`, `reeds`, or `line`). |
| `~step_size`  | `float`  | `0.1`         | Step size for generating the reference path.                       |
| `~min_radius` | `float`  | `1.0`         | Minimum turning radius for the reference path.                     |

#### Frame Configuration

| Parameter       | Type     | Default Value  | Description                       |
| --------------- | -------- | -------------- | --------------------------------- |
| `~target_frame` | `string` | `"map"`        | Target frame for transformations. |
| `~lidar_frame`  | `string` | `"lidar_link"` | Frame ID for the LiDAR sensor.    |
| `~base_frame`   | `string` | `"base_link"`  | Base frame of the robot.          |

#### Visualization

| Parameter          | Type    | Default Value | Description                                    |
| ------------------ | ------- | ------------- | ---------------------------------------------- |
| `~marker_x`        | `float` | `0.05`        | Scale factor for marker size in visualization. |
| `~marker_lifetime` | `float` | `0.1`         | Lifetime of markers in RViz (in seconds).      |


## Examples

### Dynamic collision avoidance

We provide the dynamic collision avoidance examples in Gazebo shown as follows. To run these exampreference path that the RDA planner should follow. les, please see [example/dynamic_collision_avoidance](https://github.com/hanruihua/rda_ros/tree/main/example/dynamic_collision_avoidance) for detail.



### Autonomous Driving

We provide the Autonomous Driving examples in Carla shown as follows. To run these examples, please see [example/auto_driving](https://github.com/hanruihua/rda_ros/tree/main/example/auto_driving) for detail.

https://github.com/hanruihua/rda_ros/assets/16113200/68c584eb-8a4a-4618-b0cd-d76c0ef37a1e


## Citation

If you find this code or paper helpful, please consider **starring** the repository and **citing** our work using the following BibTeX entries:

```
  @ARTICLE{10036019,
  author={Han, Ruihua and Wang, Shuai and Wang, Shuaijun and Zhang, Zeqing and Zhang, Qianru and Eldar, Yonina C. and Hao, Qi and Pan, Jia},
  journal={IEEE Robotics and Automation Letters}, 
  title={RDA: An Accelerated Collision Free Motion Planner for Autonomous Navigation in Cluttered Environments}, 
  year={2023},
  volume={8},
  number={3},
  pages={1715-1722},
  doi={10.1109/LRA.2023.3242138}}

```

```
@article{li2023edge,
  title={Edge Accelerated Robot Navigation With Collaborative Motion Planning},
  author={Li, Guoliang and Han, Ruihua and Wang, Shuai and Gao, Fei and Eldar, Yonina C and Xu, Chengzhong},
  journal={IEEE/ASME Transactions on Mechatronics},
  year={2024},
  publisher={IEEE}
}
```

## Contact

If you have any issues about the paper and code:

[Ruihua Han](https://github.com/hanruihua)

[Guoliang Li](https://github.com/GuoliangLI1998)

[Shuai Wang](https://github.com/bearswang)
