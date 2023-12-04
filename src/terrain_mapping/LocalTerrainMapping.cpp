/*
 * LocalTerrainMapping.cpp
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "terrain_mapping/LocalTerrainMapping.h"

namespace ros
{
LocalTerrainMapping::LocalTerrainMapping()
{
  pose_update_timer.start();
}

void LocalTerrainMapping::updateMap(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  pcl::fromROSMsg(*msg, *cloud);  // moveFromROSMsg is a bit faster (100 ~ 200 microsec) than fromROSMsg

  // Transform Pointcloud
  bool transform_success;
  auto transformed_cloud = pointcloud_processor_.transformPointcloud(cloud, map_.getFrameId(), transform_success);
  if (!transform_success)
    return;

  // Update Elevation map
  map_.update(*transformed_cloud);

  // Update Descriptor map
  descriptor_map_.setElevationMap(map_);
  descriptor_map_.update(*transformed_cloud);

  // Map Visualization
  grid_map_msgs::GridMap msg_gridmap;
  grid_map::GridMapRosConverter::toMessage(map_, msg_gridmap);
  elevation_map_publisher.publish(msg_gridmap);

  grid_map_msgs::GridMap msg_featuremap;
  grid_map::GridMapRosConverter::toMessage(descriptor_map_, msg_featuremap);
  descriptor_map_publisher.publish(msg_featuremap);
}

void LocalTerrainMapping::updatePose(const ros::TimerEvent& event)
{
  geometry_msgs::TransformStamped base_in_mapFrame;
  if (!transform_handler_.getTransform(frameId_map.param(), frameId_robot.param(), base_in_mapFrame))
    return;

  map_.move(grid_map::Position(base_in_mapFrame.transform.translation.x, base_in_mapFrame.transform.translation.y));
}

}  // namespace ros