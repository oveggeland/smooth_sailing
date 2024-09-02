cmake_minimum_required(VERSION 3.0.2)
project(gtsam_nav)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  rosbag
)

find_package(GTSAM REQUIRED)
find_package(PROJ REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} include)

file(GLOB navigation_src "src/*.cpp")
add_executable(navigation ${navigation_src})

target_link_libraries(navigation gtsam proj yaml-cpp ${catkin_LIBRARIES})


# Mark executable scripts (Python etc.) for installation
install(PROGRAMS
  scripts/plot_trajectory.py
  scripts/cookbook.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)