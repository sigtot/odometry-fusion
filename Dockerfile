FROM sigtot/ros-melodic-bionic-gtsam:1.0.1
RUN mkdir /workspace
WORKDIR /workspace
RUN apt-get -y upgrade
RUN apt-get install -y libtbb-dev apt-utils
COPY . .
RUN bash -c 'source /opt/ros/melodic/setup.bash && cd src && catkin_init_workspace && cd .. && catkin_make'
CMD bash -c 'source devel/setup.bash && roscore & echo waiting for roscore && sleep 3 && catkin_make run_tests && catkin_test_results'
