cmake_minimum_required(VERSION 3.0.2)
project(traversability_learning)
add_compile_options(-std=c++17)

find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf2_ros
    pcl_conversions
    height_map_core
    height_map_msgs
)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    # traversability_learning
  CATKIN_DEPENDS 
    roscpp
    tf2_ros
    pcl_conversions
    height_map_core
    height_map_msgs
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

add_executable(feature_extraction_node 
  src/feature_extraction/feature_extraction_node.cpp
  src/feature_extraction/FeatureExtraction.cpp
  src/feature_extraction/FeatureMap.cpp
)
target_link_libraries(feature_extraction_node
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)