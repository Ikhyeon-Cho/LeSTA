/*
 * ElevationMap.h
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef ELEVATION_MAP_H
#define ELEVATION_MAP_H

#include <grid_map_core/grid_map_core.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/eigen.h>

class ElevationMap : public grid_map::GridMap
{
public:
  using PointXYZR = pcl::PointXYZI;  // Intensity holds range R from the robot

  ElevationMap();
  ElevationMap(double length_x, double length_y, double grid_resolution);
  ElevationMap(const std::vector<std::string>& layers);
  void update(const pcl::PointCloud<pcl::PointXYZI>& pointcloud);

  const grid_map::GridMap::Matrix& getElevationLayer() const;
  grid_map::GridMap::Matrix& getElevationLayer();

  const grid_map::GridMap::Matrix& getVarianceLayer() const;
  grid_map::GridMap::Matrix& getVarianceLayer();

  const grid_map::GridMap::Matrix& getNumMeasuredPointsLayer() const;
  grid_map::GridMap::Matrix& getNumMeasuredPointsLayer();

  bool isEmptyAt(const grid_map::Index& index) const;
  bool isEmptyAt(const std::string& layer, const grid_map::Index& index) const;

  void smoothing();
  // void rayCasting(const grid_map::Position3 &robotPosition3);

private:  // Helper Functions
  pcl::PointCloud<PointXYZR>::Ptr getDownsampledCloudAtGrid(const pcl::PointCloud<PointXYZR>& pointcloud);

  void updateElevation(const pcl::PointCloud<PointXYZR>& pointcloud);

  void updateSampleVariance(const pcl::PointCloud<PointXYZR>& pointcloud);
};

#endif