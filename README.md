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
The node subscribes to odometries from the topic `/rovio/odometry` and publishes the optimized path on `/optimized_pose`. You can see the outgoing poses with the command
```bash
rostopic echo /optimized_pose
```

## Rviz
You can view the optimized path in rviz by adding a path visualization on the `/optimized_pose` topic.

## Export bag of optimized poses
To export a bag of optimized poses to `rosbag/paths.bag`, run
```bash
roslaunch odometry_optimizer record.launch
```

## Plot `paths.bag` with python (currently broken)
__Currently broken due to paths being replaced with poses by the publishers__

You will find python code for plotting in `/plots`.
### Setup
Install extra dependencies with
```bash
pipenv install
```

### Run
Make sure a roscore is running with
```bash
source devel/setup.bash
roscore
```
then run the python script from another terminal.
```bash
pipenv run python plot_path.py
```

## Simulating failure cases
You can simulate failure of sensors by disabling the incoming sensor streams.
Both rovio and lidar can be disabled.
To disable one of the streams, run any of the following commands:
```bash
rosservice call /toggle_rovio
rosservice call /toggle_lidar
```
As the service names imply, this toggles the streams, so to turn them on again simply run the command once more.
