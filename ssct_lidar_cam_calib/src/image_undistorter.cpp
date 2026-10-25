// Copyright 2024 Gezp (https://github.com/gezp).
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

#include "ssct_lidar_cam_calib/image_undistorter.hpp"

#include <algorithm>
#include <cmath>

namespace ssct_lidar_cam_calib
{
namespace
{

double coeff_at(const std::vector<double> & coeffs, size_t i)
{
  return i < coeffs.size() ? coeffs[i] : 0.0;
}

// theta_d = theta * (1 + k1*theta^2 + k2*theta^4 + k3*theta^6 + k4*theta^8)
double ftheta_theta_d(double theta, const std::vector<double> & coeffs)
{
  const double t2 = theta * theta;
  const double poly = 1.0 + t2 * (
    coeff_at(coeffs, 0) + t2 * (
      coeff_at(coeffs, 1) + t2 * (
        coeff_at(coeffs, 2) + t2 * coeff_at(coeffs, 3))));
  return theta * poly;
}

double ftheta_inverse(double theta_d, const std::vector<double> & coeffs)
{
  double theta = theta_d;
  for (int i = 0; i < 30; i++) {
    const double t2 = theta * theta;
    const double f = ftheta_theta_d(theta, coeffs) - theta_d;
    const double df = 1.0 + t2 * (
      3.0 * coeff_at(coeffs, 0) + t2 * (
        5.0 * coeff_at(coeffs, 1) + t2 * (
          7.0 * coeff_at(coeffs, 2) + t2 * 9.0 * coeff_at(coeffs, 3))));
    if (std::fabs(df) < 1e-12) {
      break;
    }
    const double step = f / df;
    theta = std::max(0.0, theta - step);
    if (std::fabs(step) < 1e-12) {
      break;
    }
  }
  return theta;
}

bool is_pinhole_model(const std::string & type)
{
  return type == "pinhole_radtan" || type == "pinhole";
}

bool is_ftheta_model(const std::string & type)
{
  return type == "pinhole_equidistant" || type == "equidistant" || type == "fisheye";
}

}  // namespace

bool ImageUndistorter::init(const ssct_common::CameraIntrinsicParam & source)
{
  source_ = source;
  if (source_.width <= 0 || source_.height <= 0) {
    error_message_ = "invalid source image size";
    return false;
  }
  if (!is_pinhole_model(source_.type) && !is_ftheta_model(source_.type)) {
    error_message_ = "unsupported camera model: " + source_.type;
    return false;
  }
  if (source_.intrinsics.size() < 4) {
    error_message_ = "camera model requires 4 intrinsics [fx, fy, cx, cy]";
    return false;
  }
  target_ = ssct_common::CameraIntrinsicParam();
  target_.frame_id = source_.frame_id;
  target_.width = source_.width;
  target_.height = source_.height;

  if (is_pinhole_model(source_.type)) {
    const cv::Mat K = (cv::Mat_<double>(3, 3) <<
      source_.intrinsics[0], 0.0, source_.intrinsics[2],
      0.0, source_.intrinsics[1], source_.intrinsics[3],
      0.0, 0.0, 1.0);
    cv::Mat D = cv::Mat::zeros(1, 5, CV_64F);
    for (int i = 0; i < 5 && static_cast<size_t>(i) < source_.distortion_coeffs.size(); i++) {
      D.at<double>(0, i) = source_.distortion_coeffs[i];
    }
    const cv::Size size(source_.width, source_.height);
    const cv::Mat K_new = cv::getOptimalNewCameraMatrix(K, D, size, 0.0, size);
    target_.type = "pinhole";
    target_.intrinsics = {
      K_new.at<double>(0, 0), K_new.at<double>(1, 1),
      K_new.at<double>(0, 2), K_new.at<double>(1, 2)};
    target_.distortion_coeffs.clear();
  } else {
    std::vector<double> coeffs = source_.distortion_coeffs;
    coeffs.resize(4, 0.0);
    target_.type = "ftheta";
    target_.intrinsics = {
      source_.intrinsics[0], source_.intrinsics[2], source_.intrinsics[3]};
    target_.distortion_coeffs = coeffs;
  }
  return build_maps();
}

bool ImageUndistorter::build_maps()
{
  map_x_.create(target_.height, target_.width, CV_32FC1);
  map_y_.create(target_.height, target_.width, CV_32FC1);
  bool has_valid = false;
  for (int v = 0; v < target_.height; v++) {
    float * map_x = map_x_.ptr<float>(v);
    float * map_y = map_y_.ptr<float>(v);
    for (int u = 0; u < target_.width; u++) {
      Eigen::Vector3d ray;
      double src_u = -1;
      double src_v = -1;
      if (unproject_target(u, v, ray) && project_source(ray, src_u, src_v)) {
        map_x[u] = static_cast<float>(src_u);
        map_y[u] = static_cast<float>(src_v);
        has_valid = true;
      } else {
        map_x[u] = -1.0f;
        map_y[u] = -1.0f;
      }
    }
  }
  if (!has_valid) {
    error_message_ = "failed to build undistortion map";
    return false;
  }
  return true;
}

bool ImageUndistorter::unproject_target(double u, double v, Eigen::Vector3d & ray) const
{
  if (target_.type == "pinhole") {
    const double fx = target_.intrinsics[0];
    const double fy = target_.intrinsics[1];
    const double cx = target_.intrinsics[2];
    const double cy = target_.intrinsics[3];
    ray = Eigen::Vector3d((u - cx) / fx, (v - cy) / fy, 1.0);
    return true;
  }
  const double f = target_.intrinsics[0];
  const double cx = target_.intrinsics[1];
  const double cy = target_.intrinsics[2];
  const double dx = u - cx;
  const double dy = v - cy;
  const double rho = std::hypot(dx, dy);
  if (rho < 1e-9) {
    ray = Eigen::Vector3d(0.0, 0.0, 1.0);
    return true;
  }
  const double theta = ftheta_inverse(rho / f, target_.distortion_coeffs);
  const double s = std::sin(theta);
  ray = Eigen::Vector3d(s * dx / rho, s * dy / rho, std::cos(theta));
  return true;
}

bool ImageUndistorter::project_source(const Eigen::Vector3d & ray, double & u, double & v) const
{
  if (is_pinhole_model(source_.type)) {
    const double z = ray.z();
    if (z <= 1e-6) {
      return false;
    }
    const double x = ray.x() / z;
    const double y = ray.y() / z;
    const double fx = source_.intrinsics[0];
    const double fy = source_.intrinsics[1];
    const double cx = source_.intrinsics[2];
    const double cy = source_.intrinsics[3];
    const double k1 = coeff_at(source_.distortion_coeffs, 0);
    const double k2 = coeff_at(source_.distortion_coeffs, 1);
    const double p1 = coeff_at(source_.distortion_coeffs, 2);
    const double p2 = coeff_at(source_.distortion_coeffs, 3);
    const double k3 = coeff_at(source_.distortion_coeffs, 4);
    const double r2 = x * x + y * y;
    const double radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
    const double xd = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
    const double yd = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
    u = fx * xd + cx;
    v = fy * yd + cy;
    return true;
  }
  const double f = source_.intrinsics[0];
  const double cx = source_.intrinsics[1];
  const double cy = source_.intrinsics[2];
  const double r = std::hypot(ray.x(), ray.y());
  if (r < 1e-9) {
    u = cx;
    v = cy;
    return true;
  }
  const double theta = std::atan2(r, ray.z());
  const double rho = f * ftheta_theta_d(theta, source_.distortion_coeffs);
  u = cx + rho * ray.x() / r;
  v = cy + rho * ray.y() / r;
  return true;
}

bool ImageUndistorter::process(const cv::Mat & image, cv::Mat & out_image) const
{
  if (map_x_.empty() || map_y_.empty()) {
    return false;
  }
  if (image.cols != source_.width || image.rows != source_.height) {
    return false;
  }
  cv::remap(
    image, out_image, map_x_, map_y_, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
  return true;
}

}  // namespace ssct_lidar_cam_calib
