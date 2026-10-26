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

#include <cstdint>
#include <string>
#include <vector>

#include "opencv2/core.hpp"

#include "ssct_common/calib_param/imu_intrinsic_param.hpp"
#include "ssct_common/sensor_data/imu_data.hpp"

namespace ssct_imu_intrinsic_calib
{

class ImuCalibratorInterface
{
public:
  virtual ~ImuCalibratorInterface() = default;

  // Append new samples and update progress, message, and the debug plot.
  // Returns true when the debug plot changed. It never produces a saveable
  // result.
  virtual bool add_data(const std::vector<ssct_common::ImuData> & samples) = 0;

  // Final optimization can run. This may stay true after optimize().
  virtual bool is_ready() const = 0;

  // Run the final optimization once. true means success, false means failure.
  // The caller terminates after this call.
  virtual bool optimize() = 0;

  virtual uint8_t progress() const = 0;
  virtual const std::string & message() const = 0;

  // Valid only after a successful optimize() call.
  virtual ssct_common::ImuIntrinsicParam get_result() const = 0;

  // Clear data, result, plot, and status. message() is set to the start hint.
  virtual void reset() = 0;

  virtual cv::Mat get_new_debug_image() const {return {};}
};

}  // namespace ssct_imu_intrinsic_calib
