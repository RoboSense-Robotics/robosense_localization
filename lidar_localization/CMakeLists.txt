cmake_minimum_required(VERSION 3.10)
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()
project(lidar_localization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_definitions(-DPROJECT_DIR="${PROJECT_SOURCE_DIR}")
add_definitions(-DPACKAGE_PATH="${CMAKE_CURRENT_LIST_DIR}")

### check ros_version
set(ROS_VERSION $ENV{ROS_VERSION})
message("ros_version:" $ENV{ROS_VERSION})

if(ROS_VERSION STREQUAL 1)
  # ROS1 setup
  add_definitions(-DROS1)
  configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/package_ros1.xml
    ${CMAKE_CURRENT_SOURCE_DIR}/package.xml
    COPYONLY
  )
    find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        nav_msgs
    )
    catkin_package(
        CATKIN_DEPENDS roscpp sensor_msgs nav_msgs
    )
    include_directories(
        ${catkin_INCLUDE_DIRS}
    )
    link_directories(
        ${catkin_LIBRARY_DIRS}
    )
elseif(ROS_VERSION STREQUAL 2)
    # ROS2 setup
    message("ROS2 found")
    add_definitions(-DROS2)
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/package_ros2.xml
        ${CMAKE_CURRENT_SOURCE_DIR}/package.xml
        COPYONLY
    )
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(rclcpp_components REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(nav_msgs REQUIRED)
else()
    message(FATAL_ERROR "Unsupported ROS version: ${ROS_VERSION}")
endif()

###
add_definitions(-DUSE_EIGEN_OPTIMIZATION)
add_subdirectory(lidar_matcher)
add_subdirectory(lidar_localization)
# 
add_subdirectory(node)

if(ROS_VERSION STREQUAL 2)
    ament_package()
endif()
