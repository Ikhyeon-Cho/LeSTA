cmake_minimum_required(VERSION 3.0.2)
project(traversability_mapping)
add_compile_options(-std=c++14)

find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  isr_ros_utils
  grid_map_core
  grid_map_ros
)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    # traversability_mapping
  CATKIN_DEPENDS 
    isr_ros_utils
    grid_map_core
    grid_map_ros
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
link_directories(${catkin_INCLUDE_DIRS})


add_executable(terrain_mapping_node src/terrain_mapping_node.cpp
  src/terrain_mapping/TerrainMapping.cpp
  src/terrain_mapping/ElevationMap.cpp
  src/terrain_mapping/DescriptorMap.cpp
  
)
target_link_libraries(terrain_mapping_node
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
  # isr_ros_utils
)