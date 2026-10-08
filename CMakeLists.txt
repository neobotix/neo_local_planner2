cmake_minimum_required(VERSION 3.20)
project(neo_local_planner2)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_ros_common REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(
  include
)

set(dependencies
  angles
  nav_msgs
  nav2_ros_common
  tf2
  rclcpp
  geometry_msgs
  nav2_costmap_2d
  pluginlib
  nav2_util
  nav2_core
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
  tf2_eigen
)

set(library_name neo_local_planner2)

add_library(${library_name} SHARED
        src/NeoLocalPlanner.cpp)

target_link_libraries(${library_name} PUBLIC
  angles::angles
  geometry_msgs::geometry_msgs
  nav_msgs::nav_msgs
  nav2_core::nav2_core
  nav2_costmap_2d::nav2_costmap_2d_core
  nav2_ros_common::nav2_ros_common
  nav2_util::nav2_util_core
  pluginlib::pluginlib
  rclcpp::rclcpp
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_sensor_msgs::tf2_sensor_msgs
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_eigen::tf2_eigen
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES 
  neo_local_planner2_plugin.xml
  DESTINATION share
  )

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(nav2_core neo_local_planner2_plugin.xml)

ament_package()
