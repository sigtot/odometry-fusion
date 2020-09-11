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

## Topics
The node subscribes to odometries from the topic `/rovio/odometry` and publishes optimized poses on `/optimized_pose`. You can see the outgoing poses with the ommand
```bash
rostopic echo /optimized_pose
```
