/*
 * TraversabilityPrediction.cpp
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "traversability_prediction/TraversabilityPrediction.h"

TraversabilityPrediction::TraversabilityPrediction()
{
  map_.setFrameId(map_frame);
  map_.loadTraversabilityModel(traversability_model_path_);
}

void TraversabilityPrediction::featureMapCallback(const grid_map_msgs::GridMapConstPtr& msg)
{
  // Convert msg to height map (feature map)
  grid_map::GridMapRosConverter::fromMessage(*msg, heightmap_);
  if (!is_map_initialized_)
  {
    initializeFrom(heightmap_);
    is_map_initialized_ = true;
  }

  map_.move(heightmap_.getPosition());

  map_.getHeightMatrix() = heightmap_.getHeightMatrix();
  map_.getVarianceMatrix() = heightmap_.getVarianceMatrix();
  map_.get("step") = heightmap_["step"];
  map_.get("slope") = heightmap_["slope"];
  map_.get("roughness") = heightmap_["roughness"];
  map_.get("curvature") = heightmap_["curvature"];

  map_.estimateTraversability();

  // Publish the map
  grid_map_msgs::GridMap message;
  grid_map::GridMapRosConverter::toMessage(map_, message);
  pub_traversabilitymap_.publish(message);
}

void TraversabilityPrediction::initializeFrom(const grid_map::HeightMap& featuremap)
{
  map_.setGeometry(featuremap.getLength(), featuremap.getResolution());
  map_.setFrameId(featuremap.getFrameId());

  map_.addLayer("elevation");
  map_.addLayer("variance");
  map_.addLayer("step");
  map_.addLayer("slope");
  map_.addLayer("roughness");
  map_.addLayer("curvature");
}