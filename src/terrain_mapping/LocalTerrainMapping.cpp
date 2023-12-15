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
  map_visualization_timer.start();
}

void LocalTerrainMapping::updateMap(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  pcl::fromROSMsg(*msg, *cloud);  // moveFromROSMsg is a bit faster (100 ~ 200 microsec) than fromROSMsg

  // Transform Pointcloud
  bool has_transform;
  auto transformed_cloud = pointcloud_processor_.transformPointcloud(cloud, map_.getFrameId(), has_transform);
  if (!has_transform)
    return;

  // Update Elevation map
  map_.update(*transformed_cloud);

  // Update Terrain Descriptor
  descriptor_map_.setElevationMap(map_);
  descriptor_map_.update(*transformed_cloud);
}

void LocalTerrainMapping::updatePose(const ros::TimerEvent& event)
{
  geometry_msgs::TransformStamped base_in_mapFrame;
  if (!transform_handler_.getTransform(map_FrameId.param(), base_FrameId.param(), base_in_mapFrame))
    return;

  map_.move(grid_map::Position(base_in_mapFrame.transform.translation.x, base_in_mapFrame.transform.translation.y));
}

void LocalTerrainMapping::visualize(const ros::TimerEvent& event)
{
  // Grid Map Visualization
  grid_map_msgs::GridMap msg_gridmap;
  grid_map::GridMapRosConverter::toMessage(map_, msg_gridmap);
  elevation_map_publisher.publish(msg_gridmap);

  grid_map_msgs::GridMap msg_featuremap;
  grid_map::GridMapRosConverter::toMessage(descriptor_map_, msg_featuremap);
  descriptor_map_publisher.publish(msg_featuremap);

  // Pointcloud Visualization
  sensor_msgs::PointCloud2 msg_pc;
  grid_map::GridMapRosConverter::toPointCloud(descriptor_map_, "elevation", msg_pc);
  pointcloud_map_publisher.publish(msg_pc);
}

}  // namespace ros