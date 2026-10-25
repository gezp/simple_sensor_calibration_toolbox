// Copyright 2026 Gezp (https://github.com/gezp).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ssct_lidar_lidar_calib/simple_calibrator.hpp"

#include <yaml-cpp/yaml.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>
#include <type_traits>
#include <vector>

namespace ssct_lidar_lidar_calib
{
namespace
{

using PointCloud = pcl::PointCloud<pcl::PointXYZI>;

struct Config
{
  double near_range;
  double ground_distance_threshold;
  int ground_min_points;
  double yaw_search_range;
  double yaw_step;
  int yaw_iterations;
  int normal_k_search;
  int icp_max_iterations;
  double icp_max_correspondence_distance;
};

struct Plane
{
  Eigen::Vector3d normal;
  double offset{0.0};  // plane: normal.dot(point) + offset = 0
};

bool extract_plane(
  const PointCloud::ConstPtr & cloud, const Config & config, Plane & plane,
  std::string & error)
{
  if (cloud->size() < static_cast<size_t>(config.ground_min_points)) {
    error = "too few points for ground plane extraction";
    return false;
  }
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZI> segmentation;
  segmentation.setOptimizeCoefficients(true);
  segmentation.setModelType(pcl::SACMODEL_PLANE);
  segmentation.setMethodType(pcl::SAC_RANSAC);
  segmentation.setDistanceThreshold(config.ground_distance_threshold);
  segmentation.setInputCloud(cloud);
  segmentation.segment(*inliers, *coefficients);
  if (inliers->indices.size() < static_cast<size_t>(config.ground_min_points)) {
    error = "ground plane extraction failed";
    return false;
  }
  Eigen::Vector3d normal(
    coefficients->values[0], coefficients->values[1], coefficients->values[2]);
  const double norm = normal.norm();
  if (norm < 1e-6) {
    error = "invalid ground plane";
    return false;
  }
  normal /= norm;
  double offset = coefficients->values[3] / norm;
  if (normal.z() < 0.0) {
    normal = -normal;
    offset = -offset;
  }
  if (std::fabs(normal.z()) < 1e-3) {
    error = "ground plane is not horizontal";
    return false;
  }
  plane.normal = normal;
  plane.offset = offset;
  return true;
}

PointCloud::Ptr filter_near_range(const PointCloud & cloud, double near_range)
{
  PointCloud::Ptr filtered(new PointCloud);
  filtered->reserve(cloud.size());
  for (const auto & point : cloud) {
    if (
      near_range <= 0.0 || std::fabs(point.x) >= near_range ||
      std::fabs(point.y) >= near_range || std::fabs(point.z) >= near_range)
    {
      filtered->push_back(point);
    }
  }
  return filtered;
}

PointCloud::Ptr extract_non_ground(
  const PointCloud::ConstPtr & cloud, const Plane & plane, double threshold)
{
  PointCloud::Ptr non_ground(new PointCloud);
  non_ground->reserve(cloud->size());
  for (const auto & point : *cloud) {
    const Eigen::Vector3d p(point.x, point.y, point.z);
    if (std::fabs(plane.normal.dot(p) + plane.offset) > threshold) {
      non_ground->push_back(point);
    }
  }
  return non_ground;
}

Eigen::Matrix4d yaw_transform(double yaw)
{
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return T;
}

Eigen::Matrix4d align_ground(
  const Plane & target_plane, const Plane & source_plane, const Eigen::Matrix4d & initial_guess)
{
  Eigen::Vector3d source_normal = initial_guess.block<3, 3>(0, 0) * source_plane.normal;
  Eigen::Vector3d target_normal = target_plane.normal;
  double dot = source_normal.dot(target_normal);
  if (dot < 0.0) {
    source_normal = -source_normal;
    dot = -dot;
  }
  dot = std::min(1.0, std::max(-1.0, dot));
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  if (dot < 1.0 - 1e-9) {
    Eigen::Vector3d axis = source_normal.cross(target_normal);
    if (axis.norm() < 1e-6) {
      const Eigen::Vector3d helper =
        std::fabs(target_normal.x()) < 0.9 ? Eigen::Vector3d::UnitX() :
        Eigen::Vector3d::UnitY();
      axis = target_normal.cross(helper).normalized();
    } else {
      axis.normalize();
    }
    rotation = Eigen::AngleAxisd(std::acos(dot), axis).toRotationMatrix();
  }
  // Source ground point on the z axis, moved onto the target ground plane.
  const Eigen::Vector3d source_ground(0.0, 0.0, -source_plane.offset / source_plane.normal.z());
  const Eigen::Vector3d source_in_target =
    rotation * (initial_guess.block<3, 3>(0, 0) * source_ground +
    initial_guess.block<3, 1>(0, 3));
  Eigen::Matrix4d ground_transform = Eigen::Matrix4d::Identity();
  ground_transform.block<3, 3>(0, 0) = rotation;
  ground_transform.block<3, 1>(0, 3) =
    target_normal * (-target_plane.offset - target_normal.dot(source_in_target));
  // Ground correction is expressed in the target frame, so pre-multiply.
  return ground_transform * initial_guess;
}

double registration_error(
  const pcl::KdTreeFLANN<pcl::PointXYZI> & kdtree, const PointCloud & source,
  const Eigen::Matrix4d & transform)
{
  PointCloud transformed;
  pcl::transformPointCloud(source, transformed, transform);
  double sum = 0.0;
  size_t count = 0;
  std::vector<int> indices(1);
  std::vector<float> distances(1);
  for (const auto & point : transformed) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    if (kdtree.nearestKSearch(point, 1, indices, distances) > 0) {
      sum += distances[0];
      ++count;
    }
  }
  if (count == 0) {
    return std::numeric_limits<double>::infinity();
  }
  return sum / static_cast<double>(count);
}

Eigen::Matrix4d search_yaw(
  const PointCloud::Ptr & target_non_ground, const PointCloud::Ptr & source_non_ground,
  const Eigen::Matrix4d & initial_guess, const Config & config)
{
  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  kdtree.setInputCloud(target_non_ground);
  double best_yaw = 0.0;
  double best_error = registration_error(kdtree, *source_non_ground, initial_guess);
  double range = config.yaw_search_range;
  double step = config.yaw_step;
  constexpr double deg_to_rad = 3.14159265358979323846 / 180.0;
  for (int iter = 0; iter < config.yaw_iterations; ++iter) {
    const double base_yaw = best_yaw;
    for (double delta = -range; delta <= range + 1e-9; delta += step) {
      const double yaw = base_yaw + delta * deg_to_rad;
      const double error =
        registration_error(kdtree, *source_non_ground, yaw_transform(yaw) * initial_guess);
      if (error < best_error) {
        best_error = error;
        best_yaw = yaw;
      }
    }
    range /= 2.0;
    step /= 2.0;
  }
  return yaw_transform(best_yaw) * initial_guess;
}

bool compute_normals(
  const PointCloud::ConstPtr & input, int k_search,
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr & output)
{
  if (input->empty()) {
    return false;
  }
  pcl::NormalEstimation<pcl::PointXYZI, pcl::PointXYZINormal> estimation;
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
  estimation.setSearchMethod(tree);
  estimation.setKSearch(std::min(k_search, static_cast<int>(input->size())));
  estimation.setInputCloud(input);
  estimation.compute(*output);
  for (size_t i = 0; i < output->size(); ++i) {
    (*output)[i].x = (*input)[i].x;
    (*output)[i].y = (*input)[i].y;
    (*output)[i].z = (*input)[i].z;
  }
  return true;
}

bool refine_with_icp(
  const PointCloud::ConstPtr & target, const PointCloud::ConstPtr & source,
  const Eigen::Matrix4d & initial_guess, const Config & config,
  Eigen::Matrix4d & result, std::string & error)
{
  PointCloud::Ptr source_in_target(new PointCloud);
  pcl::transformPointCloud(*source, *source_in_target, initial_guess);
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr source_normals(
    new pcl::PointCloud<pcl::PointXYZINormal>);
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr target_normals(
    new pcl::PointCloud<pcl::PointXYZINormal>);
  if (
    !compute_normals(source_in_target, config.normal_k_search, source_normals) ||
    !compute_normals(target, config.normal_k_search, target_normals))
  {
    error = "failed to compute normals";
    return false;
  }
  pcl::IterativeClosestPointWithNormals<
    pcl::PointXYZINormal, pcl::PointXYZINormal>
  icp;
  icp.setInputSource(source_normals);
  icp.setInputTarget(target_normals);
  icp.setMaximumIterations(config.icp_max_iterations);
  icp.setMaxCorrespondenceDistance(config.icp_max_correspondence_distance);
  pcl::PointCloud<pcl::PointXYZINormal> aligned;
  icp.align(aligned);
  if (!icp.hasConverged()) {
    error = "ICP did not converge";
    return false;
  }
  result = icp.getFinalTransformation().cast<double>() * initial_guess;
  return true;
}

}  // namespace

SimpleCalibrator::SimpleCalibrator(const YAML::Node & config)
{
  auto load = [&config](const char * key, auto & value) {
      if (config[key].IsDefined()) {
        value = config[key].as<std::decay_t<decltype(value)>>();
      }
    };
  load("near_range", near_range_);
  load("ground_distance_threshold", ground_distance_threshold_);
  load("ground_min_points", ground_min_points_);
  load("yaw_search_range", yaw_search_range_);
  load("yaw_step", yaw_step_);
  load("yaw_iterations", yaw_iterations_);
  load("normal_k_search", normal_k_search_);
  load("icp_max_iterations", icp_max_iterations_);
  load("icp_max_correspondence_distance", icp_max_correspondence_distance_);
  // Clamp config values so a bad yaml cannot hang or break the search.
  near_range_ = std::max(0.0, near_range_);
  ground_distance_threshold_ = std::max(1e-3, ground_distance_threshold_);
  ground_min_points_ = std::max(1, ground_min_points_);
  yaw_search_range_ = std::max(0.0, yaw_search_range_);
  yaw_step_ = std::max(1e-3, yaw_step_);
  yaw_iterations_ = std::max(0, yaw_iterations_);
  normal_k_search_ = std::max(1, normal_k_search_);
  icp_max_iterations_ = std::max(1, icp_max_iterations_);
  icp_max_correspondence_distance_ = std::max(1e-3, icp_max_correspondence_distance_);
}

bool SimpleCalibrator::calibrate(
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & target,
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & source,
  const Eigen::Matrix4d & initial_guess, Eigen::Matrix4d & result,
  std::string & error) const
{
  const Config config{
    near_range_, ground_distance_threshold_, ground_min_points_, yaw_search_range_,
    yaw_step_, yaw_iterations_, normal_k_search_, icp_max_iterations_,
    icp_max_correspondence_distance_};
  Plane target_plane;
  if (!extract_plane(target, config, target_plane, error)) {
    error = "target: " + error;
    return false;
  }
  const PointCloud::Ptr source_filtered = filter_near_range(*source, config.near_range);
  Plane source_plane;
  if (!extract_plane(source_filtered, config, source_plane, error)) {
    error = "source: " + error;
    return false;
  }
  PointCloud::Ptr target_non_ground =
    extract_non_ground(target, target_plane, config.ground_distance_threshold);
  PointCloud::Ptr source_non_ground =
    extract_non_ground(source, source_plane, config.ground_distance_threshold);
  if (
    target_non_ground->size() < static_cast<size_t>(config.ground_min_points) ||
    source_non_ground->size() < static_cast<size_t>(config.ground_min_points))
  {
    error = "too few non-ground points";
    return false;
  }
  const Eigen::Matrix4d ground_aligned =
    align_ground(target_plane, source_plane, initial_guess);
  const Eigen::Matrix4d yaw_aligned =
    search_yaw(target_non_ground, source_non_ground, ground_aligned, config);
  if (!refine_with_icp(target, source, yaw_aligned, config, result, error)) {
    return false;
  }
  if (!result.allFinite()) {
    error = "invalid calibration result";
    return false;
  }
  return true;
}

}  // namespace ssct_lidar_lidar_calib
