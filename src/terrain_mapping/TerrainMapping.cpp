/*
 * TerrainMapping.cpp
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "terrain_mapping/TerrainMapping.h"

namespace ros
{
TerrainMapping::TerrainMapping()
{
  pose_update_timer.start();
  feature_extraction_timer.start();
}

void TerrainMapping::updateMap(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  pcl::fromROSMsg(*msg, *cloud);  // moveFromROSMsg is a bit faster (100 ~ 200 microsec) than fromROSMsg

  // Transform Pointcloud
  bool transform_success;
  auto transformed_cloud = pointcloud_processor_.transformPointcloud(cloud, map_.getFrameId(), transform_success);
  if (!transform_success)
    return;

  // Update
  map_.update(*transformed_cloud);

  // Map Visualization
  grid_map_msgs::GridMap msg_gridmap;
  grid_map::GridMapRosConverter::toMessage(map_, msg_gridmap);
  elevation_map_publisher.publish(msg_gridmap);
}

void TerrainMapping::updatePose(const ros::TimerEvent& event)
{
  geometry_msgs::TransformStamped base_in_mapFrame;
  if (!transform_handler_.getTransform(frameId_map.param(), frameId_robot.param(), base_in_mapFrame))
    return;

  map_.move(grid_map::Position(base_in_mapFrame.transform.translation.x, base_in_mapFrame.transform.translation.y));
}

void TerrainMapping::extractFeatures(const ros::TimerEvent& event)
{
  map_descriptor_.update(map_);

  // Descriptor Map Visualization
  grid_map_msgs::GridMap msg;
  grid_map::GridMapRosConverter::toMessage(map_descriptor_, msg);
  descriptor_map_publisher.publish(msg);
}

}  // namespace ros