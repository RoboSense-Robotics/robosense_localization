find_package(yaml-cpp REQUIRED)
find_package(PCL REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(PROJ REQUIRED proj)
link_directories(${PROJ_LIBRARY_DIRS})
if(ROS_VERSION STREQUAL 1)
  ## ros1 配置
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    nav_msgs
    pcl_conversions
  )
  add_library(lidar_localization SHARED
    src/lidar_localization.cpp
  )
  target_include_directories(lidar_localization PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${PROJECT_SOURCE_DIR}/common
    ${PCL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(lidar_localization
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
    yaml-cpp
    lidar_matcher
    libproj.so
  )
elseif(ROS_VERSION STREQUAL 2)
  # ROS2 配置
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(pcl_conversions REQUIRED)

  add_library(lidar_localization SHARED
    src/lidar_localization.cpp
  )
  target_include_directories(lidar_localization PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${PROJECT_SOURCE_DIR}/common
    ${PCL_INCLUDE_DIRS}
  )
  target_link_libraries(lidar_localization
    ${PCL_LIBRARIES}
    yaml-cpp
    lidar_matcher
    ${PROJ_LIBRARIES}
  )
  ament_target_dependencies(lidar_localization
  rclcpp
  sensor_msgs
  nav_msgs
  pcl_conversions)

  message(STATUS "********************lidar_localization: ${CMAKE_CURRENT_LIST_DIR}/include")

  # Add install rules
  install(TARGETS lidar_localization
          LIBRARY DESTINATION lib
          ARCHIVE DESTINATION lib
          RUNTIME DESTINATION bin
  )

  install(DIRECTORY include/
          DESTINATION include
          FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  )
endif()
