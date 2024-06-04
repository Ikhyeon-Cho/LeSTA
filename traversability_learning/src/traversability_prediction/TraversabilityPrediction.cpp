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
  trav_map_.setFrameId(map_frame);
}

void TraversabilityPrediction::featureMapCallback(const grid_map_msgs::GridMapConstPtr& msg)
{
  // Convert grid map msg to grid map
  grid_map::GridMapRosConverter::fromMessage(*msg, trav_map_);

  trav_map_.update();

  // Publish the map
  grid_map_msgs::GridMap message;
  grid_map::GridMapRosConverter::toMessage(trav_map_, trav_map_.getLayers(), message);
  pub_traversabilitymap_.publish(message);
}