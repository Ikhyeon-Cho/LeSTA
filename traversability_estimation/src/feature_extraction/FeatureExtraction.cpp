/*
 * FeatureExtraction.cpp
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "feature_extraction/FeatureExtraction.h"
#include <chrono>

FeatureExtraction::FeatureExtraction()
{
  map_->setFrameId(map_frame);
  map_->setNormalEstimationRadius(normal_estimation_radius_);
}

void FeatureExtraction::HeightMapCallback(const grid_map_msgs::GridMapConstPtr& msg)
{
  // Convert grid map msg to grid map
  grid_map::GridMapRosConverter::fromMessage(*msg, *map_);

  map_->extractFeatures();

  // Publish the map
  grid_map_msgs::GridMap message;
  grid_map::GridMapRosConverter::toMessage(*map_, message);
  pub_featuremap_.publish(message);
}