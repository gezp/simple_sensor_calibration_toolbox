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

#include <array>
#include <vector>

#include <opencv2/core.hpp>

namespace ssct_imu_intrinsic_calib
{

// Allan deviation curves of both sensors.
class AllanPlot
{
public:
  void set_accel_curve(
    int axis, const std::vector<double> & tau, const std::vector<double> & adev);
  void set_gyro_curve(
    int axis, const std::vector<double> & tau, const std::vector<double> & adev);
  void set_sample_count(int sample_count);
  void set_sample_rate(double sample_rate);
  void clear_data();
  cv::Mat draw() const;

private:
  std::array<std::vector<double>, 3> accel_tau_;
  std::array<std::vector<double>, 3> accel_adev_;
  std::array<std::vector<double>, 3> gyro_tau_;
  std::array<std::vector<double>, 3> gyro_adev_;
  int sample_count_{0};
  double sample_rate_{0.0};
};

}  // namespace ssct_imu_intrinsic_calib
