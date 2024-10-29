cmake_minimum_required(VERSION 3.0.2)
project(lesta)
add_compile_options(-std=c++17)

# Define the relative path to LibTorch
set(Torch_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/libs/libtorch)
find_package(Torch REQUIRED HINTS ${Torch_DIR})

find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf2_ros
    pcl_conversions
    height_map_core
    height_map_msgs
    message_generation
)

add_service_files(
  FILES
  save_training_data.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    ${PROJECT_NAME}
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
  ${TORCH_INCLUDE_DIRS}
)

# Node: Feature extraction
add_executable(feature_extraction_node 
  src/feature_extraction/feature_extraction_node.cpp
  src/feature_extraction/FeatureExtraction.cpp
  src/feature_extraction/FeatureMap.cpp
)
target_link_libraries(feature_extraction_node
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

# Node: Label generation
add_executable(label_generation_node
  src/feature_extraction/FeatureMap.cpp
  src/label_generation/label_generation_node.cpp
  src/label_generation/LabelGeneration.cpp
)
target_link_libraries(label_generation_node
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

# Node: Traversability prediction
add_executable(traversability_prediction_node
  src/traversability_prediction/traversability_prediction_node.cpp
  src/traversability_prediction/TraversabilityPrediction.cpp
  src/traversability_prediction/TraversabilityMap.cpp
  src/traversability_prediction/classifiers/LearnedTraversability.cpp
)
target_link_libraries(traversability_prediction_node
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
  ${TORCH_LIBRARIES}
)