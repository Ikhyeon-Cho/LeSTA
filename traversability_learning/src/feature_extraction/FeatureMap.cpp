/*
 * FeatureMap.cpp
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "feature_extraction/FeatureMap.h"

namespace grid_map
{
FeatureMap::FeatureMap(const HeightMap& map) : FeatureMap(map.getLength().x(), map.getLength().y(), map.getResolution())
{
  setFrameId(map.getFrameId());
}

FeatureMap::FeatureMap(double map_length_x, double map_length_y, double resolution)
  : HeightMap(map_length_x, map_length_y, resolution)
{
  setFrameId("map");
  setGeometry(grid_map::Length(map_length_x, map_length_y), resolution);

  // Basic layers for feature map
  std::vector<std::string> feature_layers{ "step", "slope", "roughness", "curvature", "variance" };
  for (const auto& feature_layer : feature_layers)
  {
    addLayer(feature_layer, 0.0f);
    std::cout << "[@ FeatureMap] Added " << feature_layer << " layer to the height map" << std::endl;
  }
  setBasicLayers(feature_layers);

  // Normal layers for 3D normal vector visualization
  std::vector<std::string> normal_layers{ "normal_x", "normal_y", "normal_z" };
  for (const auto& normal_layer : normal_layers)
  {
    addLayer(normal_layer);
    std::cout << "[@ FeatureMap] Added " << normal_layer << " layer to the height map" << std::endl;
  }
}

bool FeatureMap::initializeFrom(const HeightMap& map)
{
  this->setFrameId(map.getFrameId());
  if (!map.hasHeightValue())
  {
    std::cout << "\033[1;32m[FeatureMap] Input height map does not have height values.\033[0m" << std::endl;
    return false;
  }

  // Copy elevation and variance layer
  this->getHeightMatrix() = map.getHeightMatrix();
  this->getVarianceMatrix() = map.getVarianceMatrix();

  return true;
}

void FeatureMap::update(double local_patch_radius)
{
  const auto& height_matrix = getHeightMatrix();
  TerrainDescriptor descriptor;
  for (grid_map::GridMapIterator iterator(*this); !iterator.isPastEnd(); ++iterator)
  {
    const size_t i = iterator.getLinearIndex();

    if (this->isEmptyAt(getHeightLayer(), *iterator))
      continue;

    if (!descriptor.principleComponentAnalysisAt(*this, *iterator))
      continue;

    this->get("step")(i) = descriptor.getStep();
    this->get("slope")(i) = descriptor.getSlope();
    this->get("roughness")(i) = descriptor.getRoughness();
    this->get("curvature")(i) = descriptor.getCurvature();

    this->get("normal_x")(i) = descriptor.getNormalVector().x();
    this->get("normal_y")(i) = descriptor.getNormalVector().y();
    this->get("normal_z")(i) = descriptor.getNormalVector().z();
  }
}
}  // namespace grid_map

void TerrainDescriptor::setLocalPatchRadius(double radius)
{
  local_radius_ = radius;
}

bool TerrainDescriptor::principleComponentAnalysisAt(const grid_map::HeightMap& map, const grid_map::Index& index)
{
  descriptor_points_.clear();

  grid_map::Position queried_cell_position;
  map.getPosition(index, queried_cell_position);

  for (grid_map::CircleIterator iterator(map, queried_cell_position, 0.15); !iterator.isPastEnd(); ++iterator)
  {
    Eigen::Vector3d point;
    if (!map.getPosition3(map.getHeightLayer(), *iterator, point))
      continue;

    descriptor_points_.push_back(point);
  }  // circle iterator ends

  if (descriptor_points_.size() < 4)
  {
    // debug
    // std::cout << "Not enough points to establish normal direction (nPoints = " << nPoints << ")" << std::endl;
    return false;
  }

  return doEigenDeconposition();
}

bool TerrainDescriptor::doEigenDeconposition()
{
  Eigen::Vector3d sum_points(Eigen::Vector3d::Zero());
  Eigen::Matrix3d squared_sum_points(Eigen::Matrix3d::Zero());
  for (const auto& point : descriptor_points_)
  {
    sum_points += point;
    squared_sum_points.noalias() += point * point.transpose();
  }
  const auto mean_points = sum_points / descriptor_points_.size();
  const auto covariance = squared_sum_points / descriptor_points_.size() - mean_points * mean_points.transpose();

  if (covariance.trace() < std::numeric_limits<float>::epsilon())
    return false;  // Covariance matrix needed for eigen decomposition is degenerated.

  trace_ = covariance.trace();

  // Compute Eigenvectors.
  // Eigenvalues are ordered small to large.
  // Worst case bound for zero eigenvalue from :
  // https://eigen.tuxfamily.org/dox/classEigen_1_1SelfAdjointEigenSolver.html
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver;
  solver.computeDirect(covariance, Eigen::DecompositionOptions::ComputeEigenvectors);

  normal_vector_ = solver.eigenvectors().col(0);
  eigen_vector2_ = solver.eigenvectors().col(1);
  eigen_vector3_ = solver.eigenvectors().col(2);
  eigen_values_ = solver.eigenvalues();

  if (eigen_values_(1) < 1e-8)  // second eigen value is near zero: Line feature
    return false;               // normal is not defined

  // Check direction of the normal vector and flip the sign towards the user defined direction.
  auto positive_Zaxis(Eigen::Vector3d::UnitZ());
  if (normal_vector_.dot(positive_Zaxis) < 0.0)
    normal_vector_ *= -1;

  return true;
}

float TerrainDescriptor::getStep() const
{
  auto minMax = std::minmax_element(descriptor_points_.begin(), descriptor_points_.end(),
                                    [](const Eigen::Vector3d& lhs, const Eigen::Vector3d& rhs) {
                                      return lhs(2) < rhs(2);  // Compare z-components.
                                    });

  double minZ = (*minMax.first)(2);   // Minimum z-component.
  double maxZ = (*minMax.second)(2);  // Maximum z-component.
  return maxZ - minZ;
}

float TerrainDescriptor::getSlope() const
{
  return std::acos(std::abs(normal_vector_.z())) * 180 / M_PI;
}

float TerrainDescriptor::getRoughness() const
{
  return std::sqrt(eigen_values_(0));
}

float TerrainDescriptor::getCurvature() const
{
  return std::abs(eigen_values_(0) / trace_);
}

const double& TerrainDescriptor::getSmallestEigenValue() const
{
  return eigen_values_(0);
}

const Eigen::Vector3d& TerrainDescriptor::getNormalVector() const
{
  return normal_vector_;
}