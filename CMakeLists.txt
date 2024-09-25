cmake_minimum_required(VERSION 3.0.2)
project(smooth_sailing)

set(CMAKE_CXX_STANDARD 17)

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

find_package(PCL REQUIRED)
find_package(Open3D REQUIRED)
find_package(GTSAM REQUIRED)
find_package(PROJ REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} include) # ${PCL_INCLUDE_DIRS} ${Open3D_INCLUDE_DIRS} include)
include_directories(${OpenCV_INCLUDE_DIRS})

file(GLOB navigation_src "src/navigation/*.cpp")
add_executable(navigation ${navigation_src})

#file(GLOB mapping_src "src/mapping/*.cpp")
add_executable(mapping src/mapping/mapping.cpp)
add_executable(image_reconstruction src/mapping/image_reconstruction.cpp)

target_link_libraries(navigation gtsam proj yaml-cpp ${catkin_LIBRARIES})
target_link_libraries(mapping gtsam yaml-cpp ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Open3D_LIBRARIES})
target_link_libraries(image_reconstruction gtsam yaml-cpp ${catkin_LIBRARIES} ${Open3D_LIBRARIES} ${OpenCV_LIBS})

# Mark executable scripts (Python etc.) for installation
install(PROGRAMS
  scripts/navigation_evaluation.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)