cmake_minimum_required(VERSION 3.0.2)
project(traversability_mapping)
add_compile_options(-std=c++14)

find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    ros_node_utils
    tf2_ros
    ros_transform_utils
    pcl_conversions
    ros_pcl_utils
    grid_map_core
    grid_map_ros
    grid_map_msgs
    nav_msgs
    visualization_msgs
)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    # traversability_mapping
  CATKIN_DEPENDS 
    roscpp
    ros_node_utils
    tf2_ros
    ros_transform_utils
    pcl_conversions
    ros_pcl_utils
    grid_map_core
    grid_map_ros
    grid_map_msgs
    nav_msgs
    visualization_msgs
)

###########
## Build ##
###########
set(CMAKE_BUILD_TYPE Release)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(local_terrain_mapping_node src/local_terrain_mapping_node.cpp
  src/terrain_mapping/LocalTerrainMapping.cpp
  src/terrain_mapping/ElevationMap.cpp
  src/terrain_mapping/DescriptorMap.cpp 
)
target_link_libraries(local_terrain_mapping_node
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(global_terrain_mapping_node src/global_terrain_mapping_node.cpp
  src/terrain_mapping/GlobalTerrainMapping.cpp
  src/terrain_mapping/ElevationMap.cpp
  src/terrain_mapping/DescriptorMap.cpp
  src/terrain_mapping/ElevationMapRosConverter.cpp
)
target_link_libraries(global_terrain_mapping_node
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(traversability_mapping_node src/traversability_mapping_node.cpp
  src/traversability_mapping/TraversabilityMapping.cpp
  src/traversability_mapping/TraversabilityMap.cpp
)
target_link_libraries(traversability_mapping_node
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)