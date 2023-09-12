cmake_minimum_required(VERSION 3.0.2)
project(arvc_gazebo_ros_plugins)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  # gazebo_ros
  pcl_conversions
  pcl_ros
)

find_package(gazebo REQUIRED)
find_package(PCL 1.3 REQUIRED)
find_package (Qt5Core REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(SDFormat REQUIRED)


link_directories(${catkin_LIBRARY_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
link_directories(${YAML_CPP_INCLUDE_DIRS})
link_directories(${Eigen_INCLUDE_DIRS})
# link_directories(${QtCore_LIBRARY_DIRS})
# link_directories(${SDFormat_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})
add_definitions(${catkin_DEFINITIONS})
add_definitions(${GAZEBO_DEFINITIONS})
# add_definitions(${SDFormat_DEFINITIONS})
add_definitions(${PCL_DEFINITIONS})
add_definitions(${YAML_CPP_DEFINITIONS})



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES arvc_gazebo_plugins
#  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  # ${SDFormat_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(arvc_gazebo_ros_dataset_generator src/arvc_gazebo_ros_dataset_generator.cpp)
# target_link_libraries(arvc_gazebo_ros_dataset_generator ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Eigen_LIBRARIES} ${PCL_LIBRARIES} ${Qt5Core_LIBRARIES} ${YAML_CPP_LIBRARIES}) 

# add_library(arvc_gazebo_ros_world_tf src/arvc_gazebo_ros_world_tf.cpp)
# target_link_libraries(arvc_gazebo_ros_world_tf ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES}) 

# add_library(arvc_gazebo_ros_model_move src/arvc_gazebo_ros_model_move.cpp)
# target_link_libraries(arvc_gazebo_ros_model_move ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${PCL_LIBRARIES} ${Qt5Core_LIBRARIES} ${YAML_CPP_LIBRARIES} ) 

# add_library(arvc_gazebo_collision_detector src/arvc_gazebo_collision_detector.cpp)
# target_link_libraries(arvc_gazebo_collision_detector ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${PCL_LIBRARIES} ${Qt5Core_LIBRARIES} ) 

add_executable(main src/main.cpp)
target_link_libraries(main ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${PCL_LIBRARIES} ${Qt5Core_LIBRARIES} ${YAML_CPP_LIBRARIES}) 