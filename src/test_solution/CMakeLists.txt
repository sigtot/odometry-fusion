cmake_minimum_required(VERSION 3.10.2)
project(test_solution)

find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test_solution
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

find_package(catkin REQUIRED COMPONENTS roscpp)
catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(test_solution test_solution.cpp)
target_link_libraries(test_solution ${catkin_LIBRARIES})
add_dependencies(test_solution ${catkin_EXPORTED_TARGETS})
