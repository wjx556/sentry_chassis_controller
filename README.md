# sentry_chassis_controller

ROS package for **RoboMaster sentry chassis** with steerable wheels.  
Features: PID velocity loop + inverse/forward kinematics + odom + tf.

## 1. Dependency
- Ubuntu 20.04 + ROS Noetic  
- `sudo apt install ros-noetic-control-toolbox`

## 2. Build
```bash
cd ~/your_catkin_ws
catkin build sentry_chassis_controller
source devel/setup.bash

3. Quick Start
roslaunch sentry_chassis_controller chassis_controller.launch

Send velocity commands:
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.5}  angular: {z: 0.0}"

4. Parameters (YAML)
Name	Type	Default	Description
wheel_base	double	0.395	front-rear distance [m]
wheel_track	double	0.3	left-right distance [m]
gains/wheel_fl/p	double	10.0	PID P for front-left wheel
5. Topics & TF

    Sub: /cmd_vel (geometry_msgs/Twist)
    Pub: /odom (nav_msgs/Odometry)
    TF: odom → base_link

6. Todo / Known Issues

    Power limit logic not implemented yet.
    Steer zero-calibration needed each boot.

7. License
MIT


--------------------------------------------------
答辩时的隐藏加分项
--------------------------------------------------
- 放一张 **rqt_graph** 截图（展示节点关系）。  
- 给一条 **调试命令** 示例：  
  ```bash
  rqt_plot /odom/twist/twist/linear/x /cmd_vel/linear/x

    写清 坐标系定义图（base_link 原点在哪，轮子顺序 FL FR RL RR）。
