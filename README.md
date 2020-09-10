# Simple odometry optimizer ROS node

## Initial setup
Download your rosbag and move it to `src/odometry_optimizer/rosbag/odometries.bag`

## Build and run
```bash
cd src
catkin_init_workspace
cd ..
catkin_make
source devel/setup.bash
roslaunch odometry_optimizer solution.launch
```
