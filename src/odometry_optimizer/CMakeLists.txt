cmake_minimum_required(VERSION 3.10)
project(odometry_optimizer)

find_package(catkin REQUIRED)
find_package(GTSAM REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
)

add_service_files(
  FILES
  ToggleEcho.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(odometry_optimizer odometry_optimizer.cpp ISAMOptimizer.cpp ISAMOptimizer.h utils.cpp utils.h)
target_link_libraries(odometry_optimizer ${catkin_LIBRARIES} ${BoostLIBRARIES} gtsam)
add_dependencies(odometry_optimizer ${catkin_EXPORTED_TARGETS})

add_executable(echo_node echo_node.cpp EchoPublisher.h)
target_link_libraries(echo_node ${catkin_LIBRARIES})
add_dependencies(echo_node ${catkin_EXPORTED_TARGETS})
