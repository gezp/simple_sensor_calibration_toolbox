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

#include "ssct_cam_imu_calib/aruco_detector.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>

namespace ssct_cam_imu_calib
{
namespace
{

cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary_from_string(const std::string & name)
{
  static const std::map<std::string, cv::aruco::PREDEFINED_DICTIONARY_NAME> dictionaries = {
    {"DICT_4X4_50", cv::aruco::DICT_4X4_50},
    {"DICT_4X4_100", cv::aruco::DICT_4X4_100},
    {"DICT_4X4_250", cv::aruco::DICT_4X4_250},
    {"DICT_4X4_1000", cv::aruco::DICT_4X4_1000},
    {"DICT_5X5_50", cv::aruco::DICT_5X5_50},
    {"DICT_5X5_100", cv::aruco::DICT_5X5_100},
    {"DICT_5X5_250", cv::aruco::DICT_5X5_250},
    {"DICT_5X5_1000", cv::aruco::DICT_5X5_1000},
    {"DICT_6X6_50", cv::aruco::DICT_6X6_50},
    {"DICT_6X6_100", cv::aruco::DICT_6X6_100},
    {"DICT_6X6_250", cv::aruco::DICT_6X6_250},
    {"DICT_6X6_1000", cv::aruco::DICT_6X6_1000},
    {"DICT_7X7_50", cv::aruco::DICT_7X7_50},
    {"DICT_7X7_100", cv::aruco::DICT_7X7_100},
    {"DICT_7X7_250", cv::aruco::DICT_7X7_250},
    {"DICT_7X7_1000", cv::aruco::DICT_7X7_1000},
    {"DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL},
  };
  auto it = dictionaries.find(name);
  if (it == dictionaries.end()) {
    return static_cast<cv::aruco::PREDEFINED_DICTIONARY_NAME>(-1);
  }
  return it->second;
}

}  // namespace

ArucoDetector::ArucoDetector(const YAML::Node & config)
{
  try {
    if (!config) {
      throw std::runtime_error("missing config");
    }
    const std::string type = config["type"] ? config["type"].as<std::string>() : "charuco";
    const std::string dictionary_name =
      config["dictionary"] ? config["dictionary"].as<std::string>() : "DICT_5X5_1000";
    const auto dictionary_id = dictionary_from_string(dictionary_name);
    if (static_cast<int>(dictionary_id) < 0) {
      throw std::runtime_error("unsupported aruco dictionary: " + dictionary_name);
    }
    dictionary_ = cv::aruco::getPredefinedDictionary(dictionary_id);
    min_corners_ = config["min_corners"] ? config["min_corners"].as<int>() : 12;
    if (type == "charuco") {
      const int squares_x = config["squares_x"] ? config["squares_x"].as<int>() : 6;
      const int squares_y = config["squares_y"] ? config["squares_y"].as<int>() : 8;
      const float square_length =
        config["square_length"] ? config["square_length"].as<float>() : 0.04f;
      const float marker_length =
        config["marker_length"] ? config["marker_length"].as<float>() : 0.03f;
      if (squares_x < 2 || squares_y < 2 || square_length <= 0.0f ||
        marker_length <= 0.0f || marker_length >= square_length)
      {
        throw std::runtime_error("invalid charuco board dimensions");
      }
      target_type_ = TargetType::CHARUCO;
      charuco_board_ = cv::aruco::CharucoBoard::create(
        squares_x, squares_y, square_length, marker_length, dictionary_);
    } else if (type == "gridboard") {
      const int markers_x = config["markers_x"] ? config["markers_x"].as<int>() : 5;
      const int markers_y = config["markers_y"] ? config["markers_y"].as<int>() : 7;
      const float marker_length =
        config["marker_length"] ? config["marker_length"].as<float>() : 0.04f;
      const float marker_separation =
        config["marker_separation"] ? config["marker_separation"].as<float>() : 0.01f;
      if (markers_x < 1 || markers_y < 1 || marker_length <= 0.0f ||
        marker_separation < 0.0f)
      {
        throw std::runtime_error("invalid gridboard dimensions");
      }
      target_type_ = TargetType::GRIDBOARD;
      grid_board_ = cv::aruco::GridBoard::create(
        markers_x, markers_y, marker_length, marker_separation, dictionary_);
    } else {
      throw std::runtime_error("unsupported target type: " + type);
    }
    max_reprojection_error_ =
      config["max_reprojection_error"] ?
      config["max_reprojection_error"].as<double>() : 2.0;
    detector_params_ = cv::aruco::DetectorParameters::create();
    detector_params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    detector_params_->cornerRefinementWinSize = 5;
    detector_params_->cornerRefinementMaxIterations = 30;
    detector_params_->cornerRefinementMinAccuracy = 0.01;
    if (config["min_marker_perimeter_rate"]) {
      detector_params_->minMarkerPerimeterRate =
        config["min_marker_perimeter_rate"].as<double>();
    }
    if (config["max_marker_perimeter_rate"]) {
      detector_params_->maxMarkerPerimeterRate =
        config["max_marker_perimeter_rate"].as<double>();
    }
    if (config["polygonal_approx_accuracy_rate"]) {
      detector_params_->polygonalApproxAccuracyRate =
        config["polygonal_approx_accuracy_rate"].as<double>();
    }
  } catch (const std::exception & e) {
    throw std::runtime_error(std::string("invalid aruco_detector config: ") + e.what());
  }
}

void ArucoDetector::set_camera_model(const CameraModel & camera_model)
{
  camera_model_ = &camera_model;
}

bool ArucoDetector::detect(const cv::Mat & image, BoardObservation & observation) const
{
  if (!camera_model_ || image.empty()) {
    return false;
  }
  cv::Mat gray;
  if (image.type() == CV_8UC1) {
    gray = image;
  } else if (image.channels() == 3) {
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  } else if (image.channels() == 4) {
    cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
  } else {
    return false;
  }
  std::vector<std::vector<cv::Point2f>> marker_corners;
  std::vector<int> marker_ids;
  bool ok = false;
  try {
    cv::aruco::detectMarkers(
      gray, dictionary_, marker_corners, marker_ids, detector_params_);
    if (marker_ids.empty()) {
      return false;
    }
    if (target_type_ == TargetType::CHARUCO) {
      ok = detect_charuco(gray, marker_corners, marker_ids, observation);
    } else {
      ok = detect_gridboard(marker_corners, marker_ids, observation);
    }
  } catch (const cv::Exception &) {
    return false;
  }
  if (!ok) {
    return false;
  }
  return estimate_pose(observation);
}

bool ArucoDetector::detect_charuco(
  const cv::Mat & image, const std::vector<std::vector<cv::Point2f>> & marker_corners,
  const std::vector<int> & marker_ids, BoardObservation & observation) const
{
  std::vector<cv::Point2f> charuco_corners;
  std::vector<int> charuco_ids;
  const int count = cv::aruco::interpolateCornersCharuco(
    marker_corners, marker_ids, image, charuco_board_, charuco_corners, charuco_ids);
  if (count < min_corners_ || count < 4) {
    return false;
  }
  observation.image_points.clear();
  observation.object_points.clear();
  const auto & board_corners = charuco_board_->chessboardCorners;
  for (size_t i = 0; i < charuco_ids.size(); i++) {
    const int id = charuco_ids[i];
    if (id < 0 || static_cast<size_t>(id) >= board_corners.size()) {
      continue;
    }
    observation.image_points.push_back(charuco_corners[i]);
    observation.object_points.push_back(board_corners[id]);
  }
  return observation.image_points.size() >= static_cast<size_t>(std::max(4, min_corners_));
}

bool ArucoDetector::detect_gridboard(
  const std::vector<std::vector<cv::Point2f>> & marker_corners,
  const std::vector<int> & marker_ids, BoardObservation & observation) const
{
  observation.image_points.clear();
  observation.object_points.clear();
  const auto & object_points = grid_board_->objPoints;
  for (size_t i = 0; i < marker_ids.size(); i++) {
    const int id = marker_ids[i];
    if (id < 0 || static_cast<size_t>(id) >= object_points.size()) {
      continue;
    }
    for (size_t j = 0; j < 4; j++) {
      observation.image_points.push_back(marker_corners[i][j]);
      observation.object_points.push_back(object_points[id][j]);
    }
  }
  return observation.image_points.size() >= static_cast<size_t>(std::max(4, min_corners_));
}

bool ArucoDetector::estimate_pose(BoardObservation & observation) const
{
  if (!camera_model_->undistort_points(
      observation.image_points, observation.normalized_points))
  {
    return false;
  }
  cv::Mat identity = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat rvec;
  cv::Mat tvec;
  try {
    cv::solvePnP(
      observation.object_points, observation.normalized_points, identity, cv::Mat(),
      rvec, tvec, false, cv::SOLVEPNP_IPPE);
    cv::solvePnPRefineLM(
      observation.object_points, observation.normalized_points, identity, cv::Mat(),
      rvec, tvec);
  } catch (const cv::Exception &) {
    return false;
  }
  cv::Mat rotation;
  cv::Rodrigues(rvec, rotation);
  observation.T_camera_board = Eigen::Matrix4d::Identity();
  for (int row = 0; row < 3; row++) {
    for (int col = 0; col < 3; col++) {
      observation.T_camera_board(row, col) = rotation.at<double>(row, col);
    }
    observation.T_camera_board(row, 3) = tvec.at<double>(row, 0);
  }
  if (!observation.T_camera_board.allFinite()) {
    return false;
  }
  std::vector<cv::Point2f> projected;
  if (!camera_model_->project_points(
      observation.object_points, observation.T_camera_board, projected))
  {
    return false;
  }
  double squared_error = 0.0;
  for (size_t i = 0; i < projected.size(); i++) {
    const cv::Point2f diff = projected[i] - observation.image_points[i];
    squared_error += diff.x * diff.x + diff.y * diff.y;
  }
  observation.reprojection_error = std::sqrt(squared_error / projected.size());
  return observation.reprojection_error <= max_reprojection_error_;
}

cv::Mat ArucoDetector::draw(
  const cv::Mat & image, const BoardObservation & observation) const
{
  if (image.empty()) {
    return {};
  }
  cv::Mat converted = image.clone();
  if (converted.depth() != CV_8U) {
    converted.convertTo(converted, CV_8U, 1.0 / 256.0);
  }
  cv::Mat rgb;
  if (converted.channels() == 3) {
    cv::cvtColor(converted, rgb, cv::COLOR_BGR2RGB);
  } else if (converted.channels() == 4) {
    cv::cvtColor(converted, rgb, cv::COLOR_BGRA2RGB);
  } else if (converted.channels() == 1) {
    cv::cvtColor(converted, rgb, cv::COLOR_GRAY2RGB);
  } else {
    return {};
  }
  for (size_t i = 0; i < observation.image_points.size(); i++) {
    cv::circle(rgb, observation.image_points[i], 3, cv::Scalar(0, 255, 0), -1);
  }
  const std::string text = observation.image_points.empty() ?
    "target not detected" :
    "corners: " + std::to_string(observation.image_points.size()) +
    ", reprojection: " + std::to_string(observation.reprojection_error) + " px";
  cv::putText(
    rgb, text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
    cv::Scalar(0, 255, 0), 2);
  return rgb;
}

}  // namespace ssct_cam_imu_calib
