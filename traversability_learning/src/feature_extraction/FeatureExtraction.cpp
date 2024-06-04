/*
 * FeatureExtraction.cpp
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "feature_extraction/FeatureExtraction.h"

FeatureExtraction::FeatureExtraction()
{
  feature_map_->setFrameId(map_frame);
  feature_map_->setFeatureExtractionRadius(normal_estimation_radius_);
}

void FeatureExtraction::HeightMapCallback(const grid_map_msgs::GridMapConstPtr& msg)
{
  // Convert grid map to grid map
  grid_map::GridMapRosConverter::fromMessage(*msg, *feature_map_);

  feature_map_->update();

  // Publish the map
  grid_map_msgs::GridMap message;
  grid_map::GridMapRosConverter::toMessage(*feature_map_, message);
  pub_featuremap_.publish(message);
}