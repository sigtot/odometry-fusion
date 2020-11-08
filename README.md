# Simple odometry optimizer ROS node
[![sigtot](https://circleci.com/gh/sigtot/ros-simple-odometry-fusion.svg?style=shield)](https://circleci.com/gh/sigtot/ros-simple-odometry-fusion)

## Initial setup
Download your rosbag and move it to `src/odometry_optimizer/rosbag/odometries.bag`

## Build and run
First, ensure catkin tools is installed (so you can `catkin build`)
```bash
sudo apt install ros-melodic-catkin python-catkin-tools
```
Then initialize the workspace and build
```
catkin init
catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Optional: Set up rovio as VO frontend
First, set up `kindr`, as explained here: https://github.com/ethz-asl/kindr.

Then do the following:
```bash
cd src
git clone git@github.com:ethz-asl/rovio.git
cd rovio
git submodule update --init --recursive # gets lightweight_filtering
cd ../..
catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

#### Run with rovio
```bash
roslaunch odometry_optimizer rovio_solution.launch
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

## Simulating failure cases
You can simulate failure of sensors by disabling the incoming sensor streams.
Both rovio and lidar can be disabled.
To disable one of the streams, run any of the following commands:
```bash
rosservice call /toggle_rovio
rosservice call /toggle_lidar
```
As the service names imply, this toggles the streams, so to turn them on again simply run the command once more.

## Running tests 2.0
Run tests with
```bash
catkin_make run_tests odometry_optimizer
```

To actually get a non-zero return code for failing tests, run the following command after running the tests
```bash
catkin_test_results
```
It will return 0 if all tests pass, and 1 otherwise.
