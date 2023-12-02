/*
 * LocalTerrainMapping.h
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef ROS_LOCAL_TERRAIN_MAPPING_H
#define ROS_LOCAL_TERRAIN_MAPPING_H

#include <ros/ros.h>

#include <isr_ros_utils/core/core.h>
#include <isr_ros_utils/tools/tools.h>

#include <sensor_msgs/PointCloud2.h>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include "terrain_mapping/ElevationMap.h"
#include "terrain_mapping/DescriptorMap.h"

namespace ros
{
class LocalTerrainMapping
{
public:
  LocalTerrainMapping();

  void updateMap(const sensor_msgs::PointCloud2ConstPtr& msg);

  void updatePose(const ros::TimerEvent& event);

  void extractFeatures(const ros::TimerEvent& event);

public:
  // ROS Parameters: Node
  roscpp::Parameter<std::string> pointcloud_topic{ "local_terrain_mapping/SubscribedTopic/pointcloud", "/velodyne_points" };
  roscpp::Parameter<std::string> elevation_map_topic{ "local_terrain_mapping/PublishingTopic/elevation_map", "map" };
  roscpp::Parameter<std::string> descriptor_map_topic{ "local_terrain_mapping/PublishingTopic/descriptor_map", "descriptor" };

  // ROS Parameters : Framd Ids
  roscpp::Parameter<std::string> frameId_robot{ "frameId_robot", "base_link" };
  roscpp::Parameter<std::string> frameId_map{ "frameId_map", "map" };

  // Elevation Map
  roscpp::Subscriber<sensor_msgs::PointCloud2> pointcloud_subscriber{ pointcloud_topic.param(),
                                                                      &LocalTerrainMapping::updateMap, this };
  roscpp::Publisher<grid_map_msgs::GridMap> elevation_map_publisher{ elevation_map_topic.param() };

  roscpp::Parameter<double> grid_resolution{ "local_terrain_mapping/ElevationMap/grid_resolution", 0.1 };
  roscpp::Parameter<double> map_length_x{ "local_terrain_mapping/ElevationMap/map_length_x", 12 };
  roscpp::Parameter<double> map_length_y{ "local_terrain_mapping/ElevationMap/map_length_y", 12 };

  // Pose update
  roscpp::Parameter<double> pose_update_duration{ "local_terrain_mapping/PoseUpdate/duration", 0.05 };  // expect 20Hz
  roscpp::Timer pose_update_timer{ pose_update_duration.param(), &LocalTerrainMapping::updatePose, this };

  // Feature extraction
  roscpp::Parameter<double> feature_extraction_duration{ "local_terrain_mapping/FeatureExtraction/duration", 0.1 };
  roscpp::Timer feature_extraction_timer{ feature_extraction_duration.param(), &LocalTerrainMapping::extractFeatures, this };
  roscpp::Publisher<grid_map_msgs::GridMap> descriptor_map_publisher{ descriptor_map_topic.param() };


private:
  ElevationMap map_{ map_length_x.param(), map_length_y.param(), grid_resolution.param() };
  DescriptorMap map_descriptor_{map_};

  TransformHandler transform_handler_;
  PointcloudProcessor<pcl::PointXYZI> pointcloud_processor_;
};

}  // namespace ros

#endif