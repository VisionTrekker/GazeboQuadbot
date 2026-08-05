find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(test_pointcloud2_layout
  test_pointcloud2_layout.cpp
)
target_include_directories(test_pointcloud2_layout PRIVATE
  ${CMAKE_SOURCE_DIR}/include
  ${GAZEBO_INCLUDE_DIRS}
)
target_link_libraries(test_pointcloud2_layout
  ${GAZEBO_LIBRARIES}
  mid360_plugin
)
ament_target_dependencies(test_pointcloud2_layout
  rclcpp
  sensor_msgs
  livox_ros_driver2
  gazebo_ros
)

ament_add_gtest(test_offset_time_monotonic
  test_offset_time_monotonic.cpp
)
target_include_directories(test_offset_time_monotonic PRIVATE
  ${CMAKE_SOURCE_DIR}/include
  ${GAZEBO_INCLUDE_DIRS}
)
target_link_libraries(test_offset_time_monotonic
  ${GAZEBO_LIBRARIES}
  mid360_plugin
)
ament_target_dependencies(test_offset_time_monotonic
  rclcpp
  sensor_msgs
  livox_ros_driver2
  gazebo_ros
)
