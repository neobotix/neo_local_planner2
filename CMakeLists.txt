cmake_minimum_required(VERSION 3.5)
project(neo_local_planner)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 14)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

set(dependencies
  rclcpp
  geometry_msgs
  nav2_costmap_2d
  pluginlib
  nav_msgs
  nav2_util
  nav2_core
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
  tf2_eigen
)

set(library_name neo_local_planner)

add_library(${library_name} SHARED
        src/NeoLocalPlanner.cpp)

ament_target_dependencies(${library_name}
  ${dependencies}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES 
  neo_local_planner_plugin.xml
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

pluginlib_export_plugin_description_file(nav2_core neo_local_planner_plugin.xml)

ament_package()



  
# find_package(catkin REQUIRED
#         COMPONENTS
#             base_local_planner
#             cmake_modules
#             costmap_2d
#             dynamic_reconfigure
#             nav_core
#             nav_msgs
#             pluginlib
#             roscpp
#             tf
#         )

# include_directories(
#     include
#     ${catkin_INCLUDE_DIRS}
# )

# ## dynamic reconfigure
# #generate_dynamic_reconfigure_options(
# #    cfg/NeoPlanner.cfg
# #)

# catkin_package(
#     INCLUDE_DIRS include
#     LIBRARIES neo_local_planner
#     CATKIN_DEPENDS
#         dynamic_reconfigure
#         nav_msgs
#         pluginlib
#         roscpp
# )

# add_library(neo_local_planner src/NeoLocalPlanner.cpp)
# add_dependencies(neo_local_planner ${catkin_EXPORTED_TARGETS})
# target_link_libraries(neo_local_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_LIBRARIES})

# install(TARGETS neo_local_planner
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        )

# install(FILES blp_plugin.xml
#     DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )
# # pluginlib_export_plugin_description_file(nav2_core blp_plugin.xml)