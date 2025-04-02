cmake_minimum_required(VERSION 3.10)
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()
project(msf_localization)


add_compile_options(-std=c++17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE "Release")
set(POLICY CMP0048 NEW)


find_package(yaml-cpp REQUIRED)
add_subdirectory(eskf)
# Detect ROS version
if(DEFINED ENV{ROS_VERSION} AND $ENV{ROS_VERSION} EQUAL 1)
  message("Detected ROS1")

  message(STATUS "=== Building for ROS1 ===")
  configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/package_ros1.xml
    ${CMAKE_CURRENT_SOURCE_DIR}/package.xml
    COPYONLY
  )
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    nav_msgs
    std_msgs
    tf2_ros
  )
  catkin_package()
  add_library(ros1_interface_lib
    ros1_interface/src/ros1_interface.cpp
  )
  target_include_directories(ros1_interface_lib PUBLIC
    ${catkin_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/ros1_interface/include
  )
  target_link_libraries(ros1_interface_lib
    ${catkin_LIBRARIES}
    eskf_lib
    glog
    ${YAML_CPP_LIBRARIES}
  )
  target_compile_definitions(ros1_interface_lib PUBLIC -DPROJECT_PATH="${CMAKE_CURRENT_LIST_DIR}/")
  set(INTERFACE_LIB ros1_interface_lib)

  add_executable(${PROJECT_NAME}_node 
    ros1_interface/src/ros1_node.cpp
  )
  target_link_libraries(${PROJECT_NAME}_node PUBLIC 
    ros1_interface_lib  
  )

elseif(DEFINED ENV{ROS_VERSION} AND $ENV{ROS_VERSION} EQUAL 2)
  message("Detected ROS2")
  message(STATUS "=== Building for ROS2 ===")
  configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/package_ros2.xml
    ${CMAKE_CURRENT_SOURCE_DIR}/package.xml
    COPYONLY
  )
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  set(dependencies
    rclcpp
    rclcpp_components
    geometry_msgs
    nav_msgs
    std_msgs
    tf2_ros
    sensor_msgs
  )

  add_library(ros2_interface_lib
    ros2_interface/src/ros2_interface.cpp
  )
  target_link_libraries(ros2_interface_lib
    eskf_lib
    glog
    stdc++fs
    ${YAML_CPP_LIBRARIES}
  )

  target_include_directories(ros2_interface_lib PUBLIC
    ${PROJECT_SOURCE_DIR}/ros2_interface/include
    ${EIGEN3_INCLUDE_DIR}
  )

  target_compile_definitions(ros2_interface_lib PUBLIC -DPROJECT_PATH="${CMAKE_CURRENT_LIST_DIR}/")
  ament_target_dependencies(ros2_interface_lib  ${dependencies})

  set(INTERFACE_LIB ros2_interface_lib)
  add_executable(${PROJECT_NAME}_node ros2_interface/src/ros2_node.cpp)
  # Use the selected interface library
  target_link_libraries(${PROJECT_NAME}_node
    ${INTERFACE_LIB}
  )
  ament_target_dependencies(${PROJECT_NAME}_node  ${dependencies})
  # Install rules
  install(
    DIRECTORY config launch 
    DESTINATION share/${PROJECT_NAME}
  )

  install(TARGETS
    ros2_interface_lib
    ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME}
  )
  ament_package()
else()
  message(FATAL_ERROR "Unsupported ROS version or ROS_VERSION not set")
endif()
