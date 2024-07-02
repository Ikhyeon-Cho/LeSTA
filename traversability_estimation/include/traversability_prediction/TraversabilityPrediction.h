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

  void initializeFrom(const grid_map::HeightMap& featuremap);

  void featureMapCallback(const grid_map_msgs::GridMapConstPtr& msg);

private:
  ros::NodeHandle nh_priv_{ "~" };

  // Frame Ids
  std::string baselink_frame{ nh_priv_.param<std::string>("/height_mapping/frame_id/base_link", "base_link") };
  std::string map_frame{ nh_priv_.param<std::string>("/height_mapping/frame_id/map", "map") };

  std::string traversability_model_path_{ nh_priv_.param<std::string>("traversabilityModel",
                                                                      "/home/ikhyeon/Downloads") };

  // ROS
  ros::Subscriber sub_featuremap_{ nh_priv_.subscribe("/traversability_estimation/features/gridmap", 10,
                                                      &TraversabilityPrediction::featureMapCallback, this) };
  ros::Publisher pub_traversabilitymap_{ nh_priv_.advertise<grid_map_msgs::GridMap>(
      "/traversability_estimation/map/gridmap", 1) };

private:
  // arbitrary values: real size will be determined by the input map
  grid_map::HeightMap heightmap_{ 10, 10, 0.1 };
  grid_map::TraversabilityMap map_{ 10, 10, 0.1 };
  bool is_map_initialized_{ false };
};

#endif  // TRAVERSABILITY_PREDICTION_H