# 定义可执行文件
add_executable(lidar_localization_node lidar_localization_node.cpp)

# 根据 ROS 版本添加特定依赖
if(ROS_VERSION STREQUAL 1)
  target_include_directories(lidar_localization_node PUBLIC
                            ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(lidar_localization_node PUBLIC
                        lidar_localization 
                        ${catkin_LIBRARIES}
  )
elseif(ROS_VERSION STREQUAL 2)
  # ROS2 的头文件和链接库通过 ament 自动处理，不需要手动指定
  ament_target_dependencies(lidar_localization_node
    rclcpp
    sensor_msgs
    nav_msgs
  )

  # 链接公共库
  target_link_libraries(lidar_localization_node
    lidar_localization
  )
endif()

# 安装目标（适用于 ROS2）
if(ROS_VERSION STREQUAL 2)
  install(TARGETS lidar_localization_node
    DESTINATION lib/${PROJECT_NAME}
  )
endif()
