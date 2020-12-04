cmake_minimum_required(VERSION 3.10)
project(odometry_optimizer)

find_package(GTSAM REQUIRED)
find_package(Boost REQUIRED COMPONENTS chrono)
find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        message_generation
        std_msgs
        tf
        tf2
        tf2_ros
        tf2_msgs
        tf2_geometry_msgs
        pcl_ros
        )

add_service_files(
        FILES
        ToggleEcho.srv
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(
        CATKIN_DEPENDS message_runtime std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(odometry_optimizer odometry_optimizer.cpp ISAMOptimizer.cpp ISAMOptimizer.h utils.cpp utils.h PoseStampedMeasurement.h QueuedPoseStampedMeasurementProcessor.cpp QueuedPoseStampedMeasurementProcessor.h IMUQueue.cpp IMUQueue.h HealthBuffer.cpp HealthBuffer.h PointCloudPublisher.cpp PointCloudPublisher.h Params.h)
target_link_libraries(odometry_optimizer gtsam ${catkin_LIBRARIES} ${PCL_LIBRARIES})


add_executable(echo_node echo_node.cpp EchoPublisher.h)
target_link_libraries(echo_node ${catkin_LIBRARIES})

add_dependencies(echo_node odometry_optimizer_generate_messages_cpp)

catkin_add_gtest(odo_tests test/odo_tests.cpp ISAMOptimizer.cpp ISAMOptimizer.h utils.cpp utils.h)
target_link_libraries(odo_tests gtsam ${catkin_LIBRARIES})
