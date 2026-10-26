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

#pragma once

#include <Eigen/Core>

#include <utility>
#include <vector>

#include <opencv2/core.hpp>

namespace ssct_imu_intrinsic_calib
{

// Accelerometer curves with the detected static intervals, following the
// imu_tk gnuplot visualization.
class ImuTkPlot
{
public:
  void set_samples(const std::vector<Eigen::Vector3d> & accel);
  void set_intervals(const std::vector<std::pair<int, int>> & intervals);
  void set_threshold_multiplier(int threshold_multiplier);
  void clear_data();
  cv::Mat draw() const;

private:
  std::vector<Eigen::Vector3d> accel_;
  std::vector<std::pair<int, int>> intervals_;  // [start, end], inclusive
  int threshold_multiplier_{0};
};

}  // namespace ssct_imu_intrinsic_calib
