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

#include <ros_node_utils/Core.h>
#include <ros_transform_utils/TransformHandler.h>
#include <ros_pcl_utils/PointcloudProcessor.h>

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

  void visualize(const ros::TimerEvent& event);

public:
  // Subscribed Topics
  roscpp::Parameter<std::string> pointcloud_topic{ "~/Subscribed_Topics/pointcloud", "/points_raw" };

  // Published Topics
  roscpp::Parameter<std::string> elevation_map_topic{ "~/Published_Topics/elevation_map", "map" };
  roscpp::Parameter<std::string> descriptor_map_topic{ "~/Published_Topics/descriptor_map", "descriptor" };
  roscpp::Parameter<std::string> pointcloud_map_topic{ "~/Published_Topics/pointcloud_map", "map_cloud" };

  // Parameters
  // -- Frame Ids
  roscpp::Parameter<std::string> base_FrameId{ "~/Parameters/base_frame_id", "base_link" };
  roscpp::Parameter<std::string> map_FrameId{ "~/Parameters/map_frame_id", "map" };
  // -- Elevation Map
  roscpp::Parameter<double> grid_resolution{ "~/Parameters/grid_resolution", 0.1 };
  roscpp::Parameter<double> map_length_x{ "~/Parameters/map_length_x", 12 };
  roscpp::Parameter<double> map_length_y{ "~/Parameters/map_length_y", 12 };
  // -- Duration
  roscpp::Parameter<double> pose_update_duration{ "~/Parameters/pose_update_duration", 0.05 };  // expect 20Hz
  roscpp::Parameter<double> map_visualization_duration{ "~/Parameters/map_visualization_duration",
                                                        0.1 };  // expect 10Hz

private:
  ElevationMap map_{ map_length_x.param(), map_length_y.param(), grid_resolution.param() };
  DescriptorMap descriptor_map_{ map_ };

  TransformHandler transform_handler_;
  PointcloudProcessor<pcl::PointXYZI> pointcloud_processor_;

  roscpp::Subscriber<sensor_msgs::PointCloud2> pointcloud_subscriber{ pointcloud_topic.param(),
                                                                      &LocalTerrainMapping::updateMap, this };
  roscpp::Publisher<grid_map_msgs::GridMap> elevation_map_publisher{ elevation_map_topic.param() };
  roscpp::Publisher<grid_map_msgs::GridMap> descriptor_map_publisher{ descriptor_map_topic.param() };
  roscpp::Publisher<sensor_msgs::PointCloud2> pointcloud_map_publisher{ pointcloud_map_topic.param() };

  roscpp::Timer pose_update_timer{ pose_update_duration.param(), &LocalTerrainMapping::updatePose, this };
  roscpp::Timer map_visualization_timer{ map_visualization_duration.param(), &LocalTerrainMapping::visualize, this };
};

}  // namespace ros

#endif