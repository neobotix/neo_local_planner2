cmake_minimum_required(VERSION 3.5)
project(neo_local_planner2)

find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 14)

include_directories(
  include
)

set(library_name neo_local_planner2)

add_library(${library_name} SHARED
        src/NeoLocalPlanner.cpp)

target_link_libraries(${library_name} PUBLIC
  rclcpp::rclcpp
  angles::angles
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  nav2_costmap_2d::nav2_costmap_2d_client
  nav2_costmap_2d::nav2_costmap_2d_core
  pluginlib::pluginlib
  nav2_core::nav2_core
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
