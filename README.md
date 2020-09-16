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
The node subscribes to odometries from the topic `/rovio/odometry` and publishes the optimized path on `/optimized_path`. You can see the outgoing poses with the command
```bash
rostopic echo /optimized_path
```

## Rviz
You can view the optimized path in rviz by adding a path visualization on the `/optimized_path` topic. 

## Export bag of optimized poses
To export a bag of optimized poses to `rosbag/paths.bag`, run
```bash
roslaunch odometry_optimizer record.launch
```

## Plot `paths.bag` with python
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
