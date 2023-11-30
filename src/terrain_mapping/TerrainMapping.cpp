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
}

void TerrainMapping::pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  pcl::fromROSMsg(*msg, *cloud);  // moveFromROSMsg is a bit faster (100 ~ 200 microsec) than fromROSMsg

  // processPointcloud(*cloud);

  map_.update(*cloud);

  grid_map_msgs::GridMap msg_gridmap;
  grid_map::GridMapRosConverter::toMessage(map_, msg_gridmap);
  elevation_map_publisher.publish(msg_gridmap);
}

// TODO: no void (return cloud)
void TerrainMapping::processPointcloud(const pcl::PointCloud<pcl::PointXYZI>& pointcloud)
{
}

void TerrainMapping::updatePose(const ros::TimerEvent& event)
{
  geometry_msgs::TransformStamped base_in_mapFrame;
  if (!transform_handler_.getTransform(frameId_map.param(), frameId_robot.param(), base_in_mapFrame))
    return;

  map_.move(grid_map::Position(base_in_mapFrame.transform.translation.x, base_in_mapFrame.transform.translation.y));
}

}  // namespace ros