/*
 * FeatureMap.h
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef TERRAIN_FEATURE_MAP_H
#define TERRAIN_FEATURE_MAP_H

#include <height_map_core/height_map_core.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

namespace grid_map
{
class FeatureMap : public HeightMap
{
public:
  using Ptr = std::shared_ptr<FeatureMap>;

  FeatureMap(const HeightMap& map);

  FeatureMap(double map_length_x, double map_length_y, double resolution);

  bool initializeFrom(const HeightMap& map);

  void setFeatureExtractionRadius(double radius);

  void update();

  // void update(std::vector<grid_map::Index> cell_indices);

private:
  double normal_estimation_radius_{ 0.15 };  // Default: 15cm radius circle as local area
};
}  // namespace grid_map

class TerrainDescriptor
{
public:
  TerrainDescriptor(double radius);

  bool principleComponentAnalysisAt(const grid_map::HeightMap& map, const grid_map::Index& index);

  float getStep() const;
  float getSlope() const;
  float getRoughness() const;
  float getCurvature() const;
  const Eigen::Vector3d& getNormalVector() const;

private:
  // helper functions
  bool doEigenDecomposition();
  const double& getSmallestEigenValue() const;

  // local patch size
  double local_radius_;

  // points in local
  std::vector<Eigen::Vector3d> descriptor_points_;

  // covariance matrix
  Eigen::Vector3d eigen_values_;
  Eigen::Vector3d normal_vector_;  // eigen_vector1_
  Eigen::Vector3d eigen_vector2_;
  Eigen::Vector3d eigen_vector3_;
  double trace_;
};

#endif  // TERRAIN_FEATURE_MAP_H