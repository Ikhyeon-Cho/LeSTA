/*
 * TraversabilityPrediction.h
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef TRAVERSABILITY_PREDICTION_H
#define TRAVERSABILITY_PREDICTION_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <height_map_core/height_map_core.h>
#include <height_map_msgs/HeightMapConverter.h>
#include <height_map_msgs/HeightMapMsgs.h>

#include "traversability_prediction/TraversabilityMap.h"

class TraversabilityPrediction
{
public:
  TraversabilityPrediction();

  void featureMapCallback(const grid_map_msgs::GridMapConstPtr& msg);

private:
  ros::NodeHandle nh_priv_{ "~" };

  // Frame Ids
  std::string baselink_frame{ nh_priv_.param<std::string>("/frame_id/base_link", "base_link") };
  std::string map_frame{ nh_priv_.param<std::string>("/frame_id/map", "map") };

  // ROS
  ros::Subscriber sub_featuremap_{ nh_priv_.subscribe("/traversability_estimation/features/gridmap", 10,
                                                      &TraversabilityPrediction::featureMapCallback, this) };
  ros::Publisher pub_traversabilitymap_{ nh_priv_.advertise<grid_map_msgs::GridMap>(
      "/traversability_estimation/map/gridmap", 1) };

private:
  // arbitrary values: real size will be determined by the input map
  grid_map::TraversabilityMap trav_map_{ 10, 10, 0.1 };
};

#endif  // TRAVERSABILITY_PREDICTION_H